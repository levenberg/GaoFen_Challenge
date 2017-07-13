// Generated by gencpp from file dji_sdk/MissionEventWpReach.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONEVENTWPREACH_H
#define DJI_SDK_MESSAGE_MISSIONEVENTWPREACH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct MissionEventWpReach_
{
  typedef MissionEventWpReach_<ContainerAllocator> Type;

  MissionEventWpReach_()
    : incident_type(0)
    , waypoint_index(0)
    , current_status(0)  {
    }
  MissionEventWpReach_(const ContainerAllocator& _alloc)
    : incident_type(0)
    , waypoint_index(0)
    , current_status(0)  {
  (void)_alloc;
    }



   typedef uint8_t _incident_type_type;
  _incident_type_type incident_type;

   typedef uint8_t _waypoint_index_type;
  _waypoint_index_type waypoint_index;

   typedef uint8_t _current_status_type;
  _current_status_type current_status;




  typedef boost::shared_ptr< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> const> ConstPtr;

}; // struct MissionEventWpReach_

typedef ::dji_sdk::MissionEventWpReach_<std::allocator<void> > MissionEventWpReach;

typedef boost::shared_ptr< ::dji_sdk::MissionEventWpReach > MissionEventWpReachPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionEventWpReach const> MissionEventWpReachConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionEventWpReach_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/root/Documents/roswork/GaoFen_Challenge/src/dji_sdk/msg', '/root/Documents/roswork/GaoFen_Challenge/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "887664b69cd341b8a21df490bb79afea";
  }

  static const char* value(const ::dji_sdk::MissionEventWpReach_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x887664b69cd341b8ULL;
  static const uint64_t static_value2 = 0xa21df490bb79afeaULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionEventWpReach";
  }

  static const char* value(const ::dji_sdk::MissionEventWpReach_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 incident_type\n\
uint8 waypoint_index\n\
uint8 current_status\n\
";
  }

  static const char* value(const ::dji_sdk::MissionEventWpReach_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.incident_type);
      stream.next(m.waypoint_index);
      stream.next(m.current_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MissionEventWpReach_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionEventWpReach_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionEventWpReach_<ContainerAllocator>& v)
  {
    s << indent << "incident_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.incident_type);
    s << indent << "waypoint_index: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.waypoint_index);
    s << indent << "current_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.current_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONEVENTWPREACH_H
