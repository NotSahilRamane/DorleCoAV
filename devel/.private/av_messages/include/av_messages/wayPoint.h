// Generated by gencpp from file av_messages/wayPoint.msg
// DO NOT EDIT!


#ifndef AV_MESSAGES_MESSAGE_WAYPOINT_H
#define AV_MESSAGES_MESSAGE_WAYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>

namespace av_messages
{
template <class ContainerAllocator>
struct wayPoint_
{
  typedef wayPoint_<ContainerAllocator> Type;

  wayPoint_()
    : position()
    , desired_velocity(0.0)  {
    }
  wayPoint_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , desired_velocity(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef double _desired_velocity_type;
  _desired_velocity_type desired_velocity;





  typedef boost::shared_ptr< ::av_messages::wayPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::av_messages::wayPoint_<ContainerAllocator> const> ConstPtr;

}; // struct wayPoint_

typedef ::av_messages::wayPoint_<std::allocator<void> > wayPoint;

typedef boost::shared_ptr< ::av_messages::wayPoint > wayPointPtr;
typedef boost::shared_ptr< ::av_messages::wayPoint const> wayPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::av_messages::wayPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::av_messages::wayPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::av_messages::wayPoint_<ContainerAllocator1> & lhs, const ::av_messages::wayPoint_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.desired_velocity == rhs.desired_velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::av_messages::wayPoint_<ContainerAllocator1> & lhs, const ::av_messages::wayPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace av_messages

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::av_messages::wayPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::av_messages::wayPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::av_messages::wayPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::av_messages::wayPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::av_messages::wayPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::av_messages::wayPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::av_messages::wayPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0a0914eff681fe061d257add5a3ce12e";
  }

  static const char* value(const ::av_messages::wayPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0a0914eff681fe06ULL;
  static const uint64_t static_value2 = 0x1d257add5a3ce12eULL;
};

template<class ContainerAllocator>
struct DataType< ::av_messages::wayPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "av_messages/wayPoint";
  }

  static const char* value(const ::av_messages::wayPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::av_messages::wayPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A standard message for local planner waypoints\n"
"\n"
"geometry_msgs/Point32 position # position in x,y for local planner and latitude longitude for global planner\n"
"\n"
"float64 desired_velocity\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::av_messages::wayPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::av_messages::wayPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.desired_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct wayPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::av_messages::wayPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::av_messages::wayPoint_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "desired_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.desired_velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AV_MESSAGES_MESSAGE_WAYPOINT_H
