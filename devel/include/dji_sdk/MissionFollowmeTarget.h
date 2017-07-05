// Generated by gencpp from file dji_sdk/MissionFollowmeTarget.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONFOLLOWMETARGET_H
#define DJI_SDK_MESSAGE_MISSIONFOLLOWMETARGET_H


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
struct MissionFollowmeTarget_
{
  typedef MissionFollowmeTarget_<ContainerAllocator> Type;

  MissionFollowmeTarget_()
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0)  {
    }
  MissionFollowmeTarget_(const ContainerAllocator& _alloc)
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0)  {
  (void)_alloc;
    }



   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef uint16_t _altitude_type;
  _altitude_type altitude;




  typedef boost::shared_ptr< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> const> ConstPtr;

}; // struct MissionFollowmeTarget_

typedef ::dji_sdk::MissionFollowmeTarget_<std::allocator<void> > MissionFollowmeTarget;

typedef boost::shared_ptr< ::dji_sdk::MissionFollowmeTarget > MissionFollowmeTargetPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionFollowmeTarget const> MissionFollowmeTargetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a0e9fef853d7437503847554a9f4ca1";
  }

  static const char* value(const ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a0e9fef853d7437ULL;
  static const uint64_t static_value2 = 0x503847554a9f4ca1ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionFollowmeTarget";
  }

  static const char* value(const ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 latitude\n\
float64 longitude\n\
uint16 altitude\n\
";
  }

  static const char* value(const ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MissionFollowmeTarget_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionFollowmeTarget_<ContainerAllocator>& v)
  {
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.altitude);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONFOLLOWMETARGET_H
