// Generated by gencpp from file carla_msgs/CarlaEgoVehicleControl.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLAEGOVEHICLECONTROL_H
#define CARLA_MSGS_MESSAGE_CARLAEGOVEHICLECONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaEgoVehicleControl_
{
  typedef CarlaEgoVehicleControl_<ContainerAllocator> Type;

  CarlaEgoVehicleControl_()
    : header()
    , throttle(0.0)
    , steer(0.0)
    , brake(0.0)
    , hand_brake(false)
    , reverse(false)
    , gear(0)
    , manual_gear_shift(false)  {
    }
  CarlaEgoVehicleControl_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , throttle(0.0)
    , steer(0.0)
    , brake(0.0)
    , hand_brake(false)
    , reverse(false)
    , gear(0)
    , manual_gear_shift(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _throttle_type;
  _throttle_type throttle;

   typedef float _steer_type;
  _steer_type steer;

   typedef float _brake_type;
  _brake_type brake;

   typedef uint8_t _hand_brake_type;
  _hand_brake_type hand_brake;

   typedef uint8_t _reverse_type;
  _reverse_type reverse;

   typedef int32_t _gear_type;
  _gear_type gear;

   typedef uint8_t _manual_gear_shift_type;
  _manual_gear_shift_type manual_gear_shift;





  typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaEgoVehicleControl_

typedef ::carla_msgs::CarlaEgoVehicleControl_<std::allocator<void> > CarlaEgoVehicleControl;

typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleControl > CarlaEgoVehicleControlPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleControl const> CarlaEgoVehicleControlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.throttle == rhs.throttle &&
    lhs.steer == rhs.steer &&
    lhs.brake == rhs.brake &&
    lhs.hand_brake == rhs.hand_brake &&
    lhs.reverse == rhs.reverse &&
    lhs.gear == rhs.gear &&
    lhs.manual_gear_shift == rhs.manual_gear_shift;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5b57fc698c12ff4c20a5fc71fba832f";
  }

  static const char* value(const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5b57fc698c12ff4ULL;
  static const uint64_t static_value2 = 0xc20a5fc71fba832fULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaEgoVehicleControl";
  }

  static const char* value(const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2018-2019 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"# This represents a vehicle control message sent to CARLA simulator\n"
"\n"
"std_msgs/Header header\n"
"\n"
"# The CARLA vehicle control data\n"
"\n"
"# 0. <= throttle <= 1.\n"
"float32 throttle\n"
"\n"
"# -1. <= steer <= 1.\n"
"float32 steer\n"
"\n"
"# 0. <= brake <= 1.\n"
"float32 brake\n"
"\n"
"# hand_brake 0 or 1\n"
"bool hand_brake\n"
"\n"
"# reverse 0 or 1\n"
"bool reverse\n"
"\n"
"# gear\n"
"int32 gear\n"
"\n"
"# manual gear shift\n"
"bool manual_gear_shift\n"
"\n"
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

  static const char* value(const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.throttle);
      stream.next(m.steer);
      stream.next(m.brake);
      stream.next(m.hand_brake);
      stream.next(m.reverse);
      stream.next(m.gear);
      stream.next(m.manual_gear_shift);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaEgoVehicleControl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaEgoVehicleControl_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "throttle: ";
    Printer<float>::stream(s, indent + "  ", v.throttle);
    s << indent << "steer: ";
    Printer<float>::stream(s, indent + "  ", v.steer);
    s << indent << "brake: ";
    Printer<float>::stream(s, indent + "  ", v.brake);
    s << indent << "hand_brake: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hand_brake);
    s << indent << "reverse: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reverse);
    s << indent << "gear: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gear);
    s << indent << "manual_gear_shift: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.manual_gear_shift);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLAEGOVEHICLECONTROL_H
