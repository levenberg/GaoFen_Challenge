// Generated by gencpp from file dji_sdk/LocalPosition.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_LOCALPOSITION_H
#define DJI_SDK_MESSAGE_LOCALPOSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace dji_sdk
{
template <class ContainerAllocator>
struct LocalPosition_
{
  typedef LocalPosition_<ContainerAllocator> Type;

  LocalPosition_()
    : header()
    , ts(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  LocalPosition_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ts(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _ts_type;
  _ts_type ts;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::dji_sdk::LocalPosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::LocalPosition_<ContainerAllocator> const> ConstPtr;

}; // struct LocalPosition_

typedef ::dji_sdk::LocalPosition_<std::allocator<void> > LocalPosition;

typedef boost::shared_ptr< ::dji_sdk::LocalPosition > LocalPositionPtr;
typedef boost::shared_ptr< ::dji_sdk::LocalPosition const> LocalPositionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::LocalPosition_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::LocalPosition_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/root/Documents/roswork/GaoFen_Challenge/src/dji_sdk/msg', '/root/Documents/roswork/GaoFen_Challenge/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::LocalPosition_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::LocalPosition_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::LocalPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::LocalPosition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::LocalPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::LocalPosition_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::LocalPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "933ce5db06b6bff36785c58a964ad3c7";
  }

  static const char* value(const ::dji_sdk::LocalPosition_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x933ce5db06b6bff3ULL;
  static const uint64_t static_value2 = 0x6785c58a964ad3c7ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::LocalPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/LocalPosition";
  }

  static const char* value(const ::dji_sdk::LocalPosition_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::LocalPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#    North(x)\n\
#   /\n\
#  /\n\
# /______East(y)\n\
# |\n\
# |\n\
# Donw (-z)\n\
Header header\n\
int32 ts\n\
float32 x\n\
float32 y\n\
float32 z \n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::dji_sdk::LocalPosition_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::LocalPosition_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ts);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct LocalPosition_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::LocalPosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::LocalPosition_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ts: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ts);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_LOCALPOSITION_H
