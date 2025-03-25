// Generated by gencpp from file state_estimator_msgs/sensor_fusion.msg
// DO NOT EDIT!


#ifndef STATE_ESTIMATOR_MSGS_MESSAGE_SENSOR_FUSION_H
#define STATE_ESTIMATOR_MSGS_MESSAGE_SENSOR_FUSION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace state_estimator_msgs
{
template <class ContainerAllocator>
struct sensor_fusion_
{
  typedef sensor_fusion_<ContainerAllocator> Type;

  sensor_fusion_()
    : header()
    , position()
    , linear_velocity()  {
      position.assign(0.0);

      linear_velocity.assign(0.0);
  }
  sensor_fusion_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position()
    , linear_velocity()  {
  (void)_alloc;
      position.assign(0.0);

      linear_velocity.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<double, 3>  _position_type;
  _position_type position;

   typedef boost::array<double, 3>  _linear_velocity_type;
  _linear_velocity_type linear_velocity;





  typedef boost::shared_ptr< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> const> ConstPtr;

}; // struct sensor_fusion_

typedef ::state_estimator_msgs::sensor_fusion_<std::allocator<void> > sensor_fusion;

typedef boost::shared_ptr< ::state_estimator_msgs::sensor_fusion > sensor_fusionPtr;
typedef boost::shared_ptr< ::state_estimator_msgs::sensor_fusion const> sensor_fusionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.position == rhs.position &&
    lhs.linear_velocity == rhs.linear_velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace state_estimator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "971096c93fdd3d0854784d51bb392049";
  }

  static const char* value(const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x971096c93fdd3d08ULL;
  static const uint64_t static_value2 = 0x54784d51bb392049ULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimator_msgs/sensor_fusion";
  }

  static const char* value(const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"float64[3] position\n"
"float64[3] linear_velocity\n"
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

  static const char* value(const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.linear_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sensor_fusion_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator_msgs::sensor_fusion_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_estimator_msgs::sensor_fusion_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "linear_velocity[]" << std::endl;
    for (size_t i = 0; i < v.linear_velocity.size(); ++i)
    {
      s << indent << "  linear_velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.linear_velocity[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MSGS_MESSAGE_SENSOR_FUSION_H
