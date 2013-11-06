#include "noise_voxel_grid.h"

#include <pcl/common/io.h>

using namespace pcl;

noise_voxel_grid::noise_voxel_grid(int min_points, int skip_points) :
    super(), min_points(min_points), skip_points(skip_points)
{

}

struct cloud_point_index_idx 
{
  unsigned int idx;
  unsigned int cloud_point_index;

  cloud_point_index_idx (unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
  bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

void noise_voxel_grid::applyFilter(PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D<PointT>(input_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D<PointT>(*input_, min_p, max_p);

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<sensor_msgs::PointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  std::vector<cloud_point_index_idx> index_vector;
  unsigned int new_size = input_->points.size() / skip_points;
  index_vector.reserve(new_size);

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<sensor_msgs::PointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (unsigned int cp = 0; cp < static_cast<unsigned int> (input_->points.size ()); ++cp)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[cp].x) || 
            !pcl_isfinite (input_->points[cp].y) || 
            !pcl_isfinite (input_->points[cp].z))
          continue;

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[cp]);
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }
      
      int ijk0 = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), cp));
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (unsigned int cp = 0; cp < static_cast<unsigned int> (input_->points.size ()); cp += skip_points)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[cp].x) || 
            !pcl_isfinite (input_->points[cp].y) || 
            !pcl_isfinite (input_->points[cp].z))
          continue;

      int ijk0 = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]) - min_b_[2]);

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), cp));
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  unsigned int total = 0; // points kept
  unsigned int index = 0;
  unsigned int filtered = 0; // points filtered out
  while (index < index_vector.size ()) 
  {
    int i = index + 1;
    int start = index;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
      ++i;
    if (i - start < min_points) { // check if enough points in voxel
      // remove points from points to add
      index_vector.erase(index_vector.begin() + start, index_vector.begin() + i); // i correct since > last
      index = start;
      ++filtered;
    }
    else { // add points
      ++total;
      index = i;
    }
  }
  
  //std::cout << "Filtered: " << float(filtered)/float(total + filtered) << std::endl;

  // Fourth pass: compute centroids, insert them into their final position
  output.points.resize (total);
  if (save_leaf_layout_)
  {
    try
    { 
      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
      uint32_t new_layout_size = div_b_[0]*div_b_[1]*div_b_[2];
      //This is the number of elements that need to be re-initialized to -1
      uint32_t reinit_size = std::min (static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
      for (uint32_t i = 0; i < reinit_size; i++)
      {
        leaf_layout_[i] = -1;
      }        
      leaf_layout_.resize (new_layout_size, -1);           
    }
    catch (std::bad_alloc&)
    {
      throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.hpp", "applyFilter");	
    }
    catch (std::length_error&)
    {
      throw PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.hpp", "applyFilter");	
    }
  }
  
  index = 0;
  Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
  Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);

  for (unsigned int cp = 0; cp < index_vector.size ();)
  {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    if (!downsample_all_data_) 
    {
      centroid[0] = input_->points[index_vector[cp].cloud_point_index].x;
      centroid[1] = input_->points[index_vector[cp].cloud_point_index].y;
      centroid[2] = input_->points[index_vector[cp].cloud_point_index].z;
    }
    else 
    {
      // ---[ RGB special case
      if (rgba_index >= 0)
      {
        // Fill r/g/b data, assuming that the order is BGRA
        pcl::RGB rgb;
        memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[index_vector[cp].cloud_point_index]) + rgba_index, sizeof (RGB));
        centroid[centroid_size-3] = rgb.r;
        centroid[centroid_size-2] = rgb.g;
        centroid[centroid_size-1] = rgb.b;
      }
      pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[index_vector[cp].cloud_point_index], centroid));
    }

    unsigned int i = cp + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx) 
    {
      if (!downsample_all_data_) 
      {
        centroid[0] += input_->points[index_vector[i].cloud_point_index].x;
        centroid[1] += input_->points[index_vector[i].cloud_point_index].y;
        centroid[2] += input_->points[index_vector[i].cloud_point_index].z;
      }
      else 
      {
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          pcl::RGB rgb;
          memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[index_vector[i].cloud_point_index]) + rgba_index, sizeof (RGB));
          temporary[centroid_size-3] = rgb.r;
          temporary[centroid_size-2] = rgb.g;
          temporary[centroid_size-1] = rgb.b;
        }
        pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[index_vector[i].cloud_point_index], temporary));
        centroid += temporary;
      }
      ++i;
    }

    // index is centroid final position in resulting PointCloud
    if (save_leaf_layout_)
      leaf_layout_[index_vector[cp].idx] = index;

    centroid /= static_cast<float> (i - cp);

    // store centroid
    // Do we need to process all the fields?
    if (!downsample_all_data_) 
    {
      output.points[index].x = centroid[0];
      output.points[index].y = centroid[1];
      output.points[index].z = centroid[2];
    }
    else 
    {
      pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor <PointT> (centroid, output.points[index]));
      // ---[ RGB special case
      if (rgba_index >= 0) 
      {
        // pack r/g/b into rgb
        float r = centroid[centroid_size-3], g = centroid[centroid_size-2], b = centroid[centroid_size-1];
        int rgb = (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
        memcpy (reinterpret_cast<char*> (&output.points[index]) + rgba_index, &rgb, sizeof (float));
      }
    }
    cp = i;
    ++index;
  }
  output.width = static_cast<uint32_t> (output.points.size ());
}
