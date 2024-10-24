// Generated by gencpp from file voronoi_cbsa/ValidSensors.msg
// DO NOT EDIT!


#ifndef VORONOI_CBSA_MESSAGE_VALIDSENSORS_H
#define VORONOI_CBSA_MESSAGE_VALIDSENSORS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace voronoi_cbsa
{
template <class ContainerAllocator>
struct ValidSensors_
{
  typedef ValidSensors_<ContainerAllocator> Type;

  ValidSensors_()
    : data()  {
    }
  ValidSensors_(const ContainerAllocator& _alloc)
    : data(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> const> ConstPtr;

}; // struct ValidSensors_

typedef ::voronoi_cbsa::ValidSensors_<std::allocator<void> > ValidSensors;

typedef boost::shared_ptr< ::voronoi_cbsa::ValidSensors > ValidSensorsPtr;
typedef boost::shared_ptr< ::voronoi_cbsa::ValidSensors const> ValidSensorsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::voronoi_cbsa::ValidSensors_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::voronoi_cbsa::ValidSensors_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::ValidSensors_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::voronoi_cbsa::ValidSensors_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::ValidSensors_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace voronoi_cbsa

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cce5a364f3a3be12c9722c6dcad2fa94";
  }

  static const char* value(const ::voronoi_cbsa::ValidSensors_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcce5a364f3a3be12ULL;
  static const uint64_t static_value2 = 0xc9722c6dcad2fa94ULL;
};

template<class ContainerAllocator>
struct DataType< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "voronoi_cbsa/ValidSensors";
  }

  static const char* value(const ::voronoi_cbsa::ValidSensors_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] data\n"
;
  }

  static const char* value(const ::voronoi_cbsa::ValidSensors_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ValidSensors_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::voronoi_cbsa::ValidSensors_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::voronoi_cbsa::ValidSensors_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VORONOI_CBSA_MESSAGE_VALIDSENSORS_H
