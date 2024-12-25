// Generated by gencpp from file voronoi_cbsa/ExchangeData.msg
// DO NOT EDIT!


#ifndef VORONOI_CBSA_MESSAGE_EXCHANGEDATA_H
#define VORONOI_CBSA_MESSAGE_EXCHANGEDATA_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <voronoi_cbsa/SensorArray.h>
#include <voronoi_cbsa/WeightArray.h>
#include <voronoi_cbsa/WeightArray.h>
#include <geometry_msgs/Point.h>

namespace voronoi_cbsa
{
template <class ContainerAllocator>
struct ExchangeData_
{
  typedef ExchangeData_<ContainerAllocator> Type;

  ExchangeData_()
    : id(0)
    , position()
    , role()
    , weights()
    , sensor_scores()
    , operation_range(0.0)
    , approx_param(0.0)
    , smoke_variance(0.0)
    , camera_range(0.0)
    , angle_of_view(0.0)
    , camera_variance(0.0)
    , velocity()  {
    }
  ExchangeData_(const ContainerAllocator& _alloc)
    : id(0)
    , position(_alloc)
    , role(_alloc)
    , weights(_alloc)
    , sensor_scores(_alloc)
    , operation_range(0.0)
    , approx_param(0.0)
    , smoke_variance(0.0)
    , camera_range(0.0)
    , angle_of_view(0.0)
    , camera_variance(0.0)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef int64_t _id_type;
  _id_type id;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::voronoi_cbsa::SensorArray_<ContainerAllocator>  _role_type;
  _role_type role;

   typedef  ::voronoi_cbsa::WeightArray_<ContainerAllocator>  _weights_type;
  _weights_type weights;

   typedef  ::voronoi_cbsa::WeightArray_<ContainerAllocator>  _sensor_scores_type;
  _sensor_scores_type sensor_scores;

   typedef double _operation_range_type;
  _operation_range_type operation_range;

   typedef double _approx_param_type;
  _approx_param_type approx_param;

   typedef double _smoke_variance_type;
  _smoke_variance_type smoke_variance;

   typedef double _camera_range_type;
  _camera_range_type camera_range;

   typedef double _angle_of_view_type;
  _angle_of_view_type angle_of_view;

   typedef double _camera_variance_type;
  _camera_variance_type camera_variance;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> const> ConstPtr;

}; // struct ExchangeData_

typedef ::voronoi_cbsa::ExchangeData_<std::allocator<void> > ExchangeData;

typedef boost::shared_ptr< ::voronoi_cbsa::ExchangeData > ExchangeDataPtr;
typedef boost::shared_ptr< ::voronoi_cbsa::ExchangeData const> ExchangeDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::voronoi_cbsa::ExchangeData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::voronoi_cbsa::ExchangeData_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::ExchangeData_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.position == rhs.position &&
    lhs.role == rhs.role &&
    lhs.weights == rhs.weights &&
    lhs.sensor_scores == rhs.sensor_scores &&
    lhs.operation_range == rhs.operation_range &&
    lhs.approx_param == rhs.approx_param &&
    lhs.smoke_variance == rhs.smoke_variance &&
    lhs.camera_range == rhs.camera_range &&
    lhs.angle_of_view == rhs.angle_of_view &&
    lhs.camera_variance == rhs.camera_variance &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::voronoi_cbsa::ExchangeData_<ContainerAllocator1> & lhs, const ::voronoi_cbsa::ExchangeData_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace voronoi_cbsa

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f42667d1e0a184c7cc115f894849626";
  }

  static const char* value(const ::voronoi_cbsa::ExchangeData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f42667d1e0a184cULL;
  static const uint64_t static_value2 = 0x7cc115f894849626ULL;
};

template<class ContainerAllocator>
struct DataType< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "voronoi_cbsa/ExchangeData";
  }

  static const char* value(const ::voronoi_cbsa::ExchangeData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64               id\n"
"geometry_msgs/Point position\n"
"SensorArray         role\n"
"WeightArray         weights\n"
"WeightArray         sensor_scores\n"
"float64             operation_range\n"
"float64             approx_param\n"
"float64             smoke_variance\n"
"float64             camera_range\n"
"float64             angle_of_view\n"
"float64             camera_variance\n"
"geometry_msgs/Point velocity\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: voronoi_cbsa/SensorArray\n"
"Sensor[] sensors\n"
"================================================================================\n"
"MSG: voronoi_cbsa/Sensor\n"
"string type\n"
"float64 score\n"
"================================================================================\n"
"MSG: voronoi_cbsa/WeightArray\n"
"Weight[] weights\n"
"================================================================================\n"
"MSG: voronoi_cbsa/Weight\n"
"string  type\n"
"int16   event_id\n"
"float64 score\n"
;
  }

  static const char* value(const ::voronoi_cbsa::ExchangeData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.position);
      stream.next(m.role);
      stream.next(m.weights);
      stream.next(m.sensor_scores);
      stream.next(m.operation_range);
      stream.next(m.approx_param);
      stream.next(m.smoke_variance);
      stream.next(m.camera_range);
      stream.next(m.angle_of_view);
      stream.next(m.camera_variance);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ExchangeData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::voronoi_cbsa::ExchangeData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::voronoi_cbsa::ExchangeData_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.id);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "role: ";
    s << std::endl;
    Printer< ::voronoi_cbsa::SensorArray_<ContainerAllocator> >::stream(s, indent + "  ", v.role);
    s << indent << "weights: ";
    s << std::endl;
    Printer< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >::stream(s, indent + "  ", v.weights);
    s << indent << "sensor_scores: ";
    s << std::endl;
    Printer< ::voronoi_cbsa::WeightArray_<ContainerAllocator> >::stream(s, indent + "  ", v.sensor_scores);
    s << indent << "operation_range: ";
    Printer<double>::stream(s, indent + "  ", v.operation_range);
    s << indent << "approx_param: ";
    Printer<double>::stream(s, indent + "  ", v.approx_param);
    s << indent << "smoke_variance: ";
    Printer<double>::stream(s, indent + "  ", v.smoke_variance);
    s << indent << "camera_range: ";
    Printer<double>::stream(s, indent + "  ", v.camera_range);
    s << indent << "angle_of_view: ";
    Printer<double>::stream(s, indent + "  ", v.angle_of_view);
    s << indent << "camera_variance: ";
    Printer<double>::stream(s, indent + "  ", v.camera_variance);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VORONOI_CBSA_MESSAGE_EXCHANGEDATA_H
