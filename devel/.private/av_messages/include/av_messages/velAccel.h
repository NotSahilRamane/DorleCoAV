// Generated by gencpp from file av_messages/velAccel.msg
// DO NOT EDIT!


#ifndef AV_MESSAGES_MESSAGE_VELACCEL_H
#define AV_MESSAGES_MESSAGE_VELACCEL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose2D.h>

namespace av_messages
{
template <class ContainerAllocator>
struct velAccel_
{
  typedef velAccel_<ContainerAllocator> Type;

  velAccel_()
    : header()
    , vel()
    , accel()  {
    }
  velAccel_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vel(_alloc)
    , accel(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _vel_type;
  _vel_type vel;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _accel_type;
  _accel_type accel;





  typedef boost::shared_ptr< ::av_messages::velAccel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::av_messages::velAccel_<ContainerAllocator> const> ConstPtr;

}; // struct velAccel_

typedef ::av_messages::velAccel_<std::allocator<void> > velAccel;

typedef boost::shared_ptr< ::av_messages::velAccel > velAccelPtr;
typedef boost::shared_ptr< ::av_messages::velAccel const> velAccelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::av_messages::velAccel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::av_messages::velAccel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::av_messages::velAccel_<ContainerAllocator1> & lhs, const ::av_messages::velAccel_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vel == rhs.vel &&
    lhs.accel == rhs.accel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::av_messages::velAccel_<ContainerAllocator1> & lhs, const ::av_messages::velAccel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace av_messages

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::av_messages::velAccel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::av_messages::velAccel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::av_messages::velAccel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::av_messages::velAccel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::av_messages::velAccel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::av_messages::velAccel_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::av_messages::velAccel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3636f8484fe5af4c13afe61d470942f7";
  }

  static const char* value(const ::av_messages::velAccel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3636f8484fe5af4cULL;
  static const uint64_t static_value2 = 0x13afe61d470942f7ULL;
};

template<class ContainerAllocator>
struct DataType< ::av_messages::velAccel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "av_messages/velAccel";
  }

  static const char* value(const ::av_messages::velAccel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::av_messages::velAccel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Vehicle Longitudinal Velocity and Acceleration av_messages\n"
"std_msgs/Header header\n"
"\n"
"geometry_msgs/Pose2D vel # Velocity in x, y, theta\n"
"geometry_msgs/Pose2D accel # Acceleration in x, y, theta\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::av_messages::velAccel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::av_messages::velAccel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vel);
      stream.next(m.accel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct velAccel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::av_messages::velAccel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::av_messages::velAccel_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "accel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.accel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AV_MESSAGES_MESSAGE_VELACCEL_H