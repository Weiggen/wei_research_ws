// Generated by gencpp from file state_estimation/Int32MultiArrayStamped.msg
// DO NOT EDIT!


#ifndef STATE_ESTIMATION_MESSAGE_INT32MULTIARRAYSTAMPED_H
#define STATE_ESTIMATION_MESSAGE_INT32MULTIARRAYSTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace state_estimation
{
template <class ContainerAllocator>
struct Int32MultiArrayStamped_
{
  typedef Int32MultiArrayStamped_<ContainerAllocator> Type;

  Int32MultiArrayStamped_()
    : header()
    , data()  {
    }
  Int32MultiArrayStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> const> ConstPtr;

}; // struct Int32MultiArrayStamped_

typedef ::state_estimation::Int32MultiArrayStamped_<std::allocator<void> > Int32MultiArrayStamped;

typedef boost::shared_ptr< ::state_estimation::Int32MultiArrayStamped > Int32MultiArrayStampedPtr;
typedef boost::shared_ptr< ::state_estimation::Int32MultiArrayStamped const> Int32MultiArrayStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator1> & lhs, const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator1> & lhs, const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace state_estimation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1ce4762ce13f3d9e1f17586acc253067";
  }

  static const char* value(const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1ce4762ce13f3d9eULL;
  static const uint64_t static_value2 = 0x1f17586acc253067ULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimation/Int32MultiArrayStamped";
  }

  static const char* value(const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int32[] data\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Int32MultiArrayStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_estimation::Int32MultiArrayStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATION_MESSAGE_INT32MULTIARRAYSTAMPED_H
