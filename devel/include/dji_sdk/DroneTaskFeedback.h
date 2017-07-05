// Generated by gencpp from file dji_sdk/DroneTaskFeedback.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_DRONETASKFEEDBACK_H
#define DJI_SDK_MESSAGE_DRONETASKFEEDBACK_H


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
struct DroneTaskFeedback_
{
  typedef DroneTaskFeedback_<ContainerAllocator> Type;

  DroneTaskFeedback_()
    : progress(0)  {
    }
  DroneTaskFeedback_(const ContainerAllocator& _alloc)
    : progress(0)  {
  (void)_alloc;
    }



   typedef uint8_t _progress_type;
  _progress_type progress;




  typedef boost::shared_ptr< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct DroneTaskFeedback_

typedef ::dji_sdk::DroneTaskFeedback_<std::allocator<void> > DroneTaskFeedback;

typedef boost::shared_ptr< ::dji_sdk::DroneTaskFeedback > DroneTaskFeedbackPtr;
typedef boost::shared_ptr< ::dji_sdk::DroneTaskFeedback const> DroneTaskFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d0ccee79f15d1d61b42a87d5f604edbc";
  }

  static const char* value(const ::dji_sdk::DroneTaskFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd0ccee79f15d1d61ULL;
  static const uint64_t static_value2 = 0xb42a87d5f604edbcULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/DroneTaskFeedback";
  }

  static const char* value(const ::dji_sdk::DroneTaskFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
uint8 progress \n\
\n\
";
  }

  static const char* value(const ::dji_sdk::DroneTaskFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.progress);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DroneTaskFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::DroneTaskFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::DroneTaskFeedback_<ContainerAllocator>& v)
  {
    s << indent << "progress: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.progress);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_DRONETASKFEEDBACK_H
