// Generated by gencpp from file voronoi_cbsa/WeightArray.msg
// DO NOT EDIT!


#ifndef VORONOI_CBSA_MESSAGE_WEIGHTARRAY_H
#define VORONOI_CBSA_MESSAGE_WEIGHTARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <voronoi_cbsa/Weight.h>

namespace voronoi_cbsa
{
template <class ContainerAllocator>
struct WeightArray_
{
  typedef WeightArray_<ContainerAllocator> Type;

  WeightArray_()
    : weights()  {
    }
  WeightArray_(const ContainerAllocator& _alloc)
    : weights(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::voronoi_cbsa::Weight_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::voronoi_cbsa::Weight_<ContainerAllocator> >> _weights_type;
  _weights_type weights;





  typedef boost::shared_ptr< ::voronoi_cbsa::WeightArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::voronoi_cbsa::WeightArray_<ContainerAllocator> const> ConstPtr;

}; // struct WeightArray_

typedef ::voronoi_cbsa::WeightArray_<std::allocator<void> > WeightArray;

typedef boost::shared_ptr< ::voronoi_cbsa::WeightArray > WeightArrayPtr;
typedef boost::shared_ptr< ::voronoi_cbsa::WeightArray const> WeightArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::voronoi_cbsa::WeightArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::voronoi_cbsa::WeightArray_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::WeightArray_<ContainerAllocator2> & rhs)
{
  return lhs.weights == rhs.weights;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::voronoi_cbsa::WeightArray_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::WeightArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace voronoi_cbsa

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::WeightArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::WeightArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::WeightArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a52d516a208ee351d816a3ad44a096ad";
  }

  static const char* value(const ::voronoi_cbsa::WeightArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa52d516a208ee351ULL;
  static const uint64_t static_value2 = 0xd816a3ad44a096adULL;
};

template<class ContainerAllocator>
struct DataType< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "voronoi_cbsa/WeightArray";
  }

  static const char* value(const ::voronoi_cbsa::WeightArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Weight[] weights\n"
"================================================================================\n"
"MSG: voronoi_cbsa/Weight\n"
"string  type\n"
"int16   event_id\n"
"float64 score\n"
;
  }

  static const char* value(const ::voronoi_cbsa::WeightArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.weights);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WeightArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::voronoi_cbsa::WeightArray_<ContainerAllocator>& v)
  {
    s << indent << "weights[]" << std::endl;
    for (size_t i = 0; i < v.weights.size(); ++i)
    {
      s << indent << "  weights[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::voronoi_cbsa::Weight_<ContainerAllocator> >::stream(s, indent + "    ", v.weights[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VORONOI_CBSA_MESSAGE_WEIGHTARRAY_H
