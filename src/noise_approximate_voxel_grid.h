#ifndef NOISE_APPROXIMATE_VOXEL_GRID_H
#define NOISE_APPROXIMATE_VOXEL_GRID_H

#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <boost/mpl/size.hpp>

/*namespace pcl
{
  // Helper functor structure for copying data between an Eigen::VectorXf and a PointT.
  template <typename PointT>
  struct xNdCopyEigenPointFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;
    
    xNdCopyEigenPointFunctor (const Eigen::VectorXf &p1, PointT &p2)
      : p1_ (p1),
        p2_ (reinterpret_cast<Pod&>(p2)),
        f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //boost::fusion::at_key<Key> (p2_) = p1_[f_idx_++];
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&p2_) + pcl::traits::offset<PointT, Key>::value;
      *reinterpret_cast<T*>(data_ptr) = static_cast<T> (p1_[f_idx_++]);
    }

    private:
      const Eigen::VectorXf &p1_;
      Pod &p2_;
      int f_idx_;
  };

  // Helper functor structure for copying data between an Eigen::VectorXf and a PointT.
  template <typename PointT>
  struct xNdCopyPointEigenFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;
    
    xNdCopyPointEigenFunctor (const PointT &p1, Eigen::VectorXf &p2)
      : p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //p2_[f_idx_++] = boost::fusion::at_key<Key> (p1_);
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&p1_) + pcl::traits::offset<PointT, Key>::value;
      p2_[f_idx_++] = static_cast<float> (*reinterpret_cast<const T*>(data_ptr));
    }

    private:
      const Pod &p1_;
      Eigen::VectorXf &p2_;
      int f_idx_;
  };
}*/

/** \brief ApproximateVoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
*
* \author James Bowman, Radu B. Rusu
* \ingroup filters
*/
class noise_approximate_voxel_grid : public pcl::Filter<pcl::PointXYZ>
{
  typedef pcl::PointXYZ PointT;

  using pcl::Filter<PointT>::filter_name_;
  using pcl::Filter<PointT>::getClassName;
  using pcl::Filter<PointT>::input_;
  using pcl::Filter<PointT>::indices_;

  typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

private:
  struct he
  {
    he () : ix (), iy (), iz (), count (0), centroid () {}
    int ix, iy, iz;
    int count;
    Eigen::VectorXf centroid;
  };

public:
  /** \brief Empty constructor. */
  noise_approximate_voxel_grid (int min_points, int skip_points) : 
    pcl::Filter<PointT> (),
    leaf_size_ (Eigen::Vector3f::Ones ()),
    inverse_leaf_size_ (Eigen::Array3f::Ones ()),
    downsample_all_data_ (true), histsize_ (512),
    history_ (new he[histsize_]),
    min_points(min_points),
    skip_points(skip_points)
  {
    filter_name_ = "ApproximateVoxelGrid";
  }

  /** \brief Copy constructor. 
    * \param[in] src the approximate voxel grid to copy into this. 
    */
  noise_approximate_voxel_grid (const noise_approximate_voxel_grid &src) : 
    pcl::Filter<PointT> (),
    leaf_size_ (src.leaf_size_),
    inverse_leaf_size_ (src.inverse_leaf_size_),
    downsample_all_data_ (src.downsample_all_data_), 
    histsize_ (src.histsize_),
    history_ (),
    min_points(src.min_points),
    skip_points(src.skip_points)
  {
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
  }

  /** \brief Copy operator. 
    * \param[in] src the approximate voxel grid to copy into this. 
    */
  inline noise_approximate_voxel_grid& 
  operator = (const noise_approximate_voxel_grid &src)
  {
    leaf_size_ = src.leaf_size_;
    inverse_leaf_size_ = src.inverse_leaf_size_;
    downsample_all_data_ = src.downsample_all_data_;
    histsize_ = src.histsize_;
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
    min_points = src.min_points;
    skip_points = src.skip_points;
    return (*this);
  }

  /** \brief Set the voxel grid leaf size.
    * \param[in] leaf_size the voxel grid leaf size
    */
  inline void 
  setLeafSize (const Eigen::Vector3f &leaf_size) 
  { 
    leaf_size_ = leaf_size; 
    inverse_leaf_size_ = Eigen::Array3f::Ones () / leaf_size_.array ();
  }

  /** \brief Set the voxel grid leaf size.
    * \param[in] lx the leaf size for X
    * \param[in] ly the leaf size for Y
    * \param[in] lz the leaf size for Z
    */
  inline void
  setLeafSize (float lx, float ly, float lz)
  {
    setLeafSize (Eigen::Vector3f (lx, ly, lz));
  }

  /** \brief Get the voxel grid leaf size. */
  inline Eigen::Vector3f 
  getLeafSize () const { return (leaf_size_); }

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
    * \param downsample the new value (true/false)
    */
  inline void 
  setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

  /** \brief Get the state of the internal downsampling parameter (true if
    * all fields need to be downsampled, false if just XYZ). 
    */
  inline bool 
  getDownsampleAllData () const { return (downsample_all_data_); }

protected:
  /** \brief The size of a leaf. */
  Eigen::Vector3f leaf_size_;

  /** \brief Compute 1/leaf_size_ to avoid division later */ 
  Eigen::Array3f inverse_leaf_size_;

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_;

  /** \brief history buffer size, power of 2 */
  size_t histsize_;

  /** \brief history buffer */
  struct he* history_;
  
  int min_points;
  
  int skip_points;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param output the resultant point cloud message
    */
  void 
  applyFilter (PointCloud &output);

  /** \brief Write a single point from the hash to the output cloud
    */
  void 
  flush(PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size);
};

/*class noise_approximate_voxel_grid : public pcl::ApproximateVoxelGrid<pcl::PointXYZ> {
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::ApproximateVoxelGrid<PointT> super;
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
private:
    struct he
    {
        he () : ix (), iy (), iz (), count (0), centroid () {}
        int ix, iy, iz;
        int count;
        Eigen::VectorXf centroid;
    };
protected:
    using super::filter_name_;
    using super::getClassName;
    using super::input_;
    using super::indices_;
    void flush (PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size);
    void applyFilter(PointCloud &output);
private:
    int min_points;
public:
    noise_approximate_voxel_grid(int min_points);
    
};*/
#endif // NOISE_APPROXIMATE_VOXEL_GRID
