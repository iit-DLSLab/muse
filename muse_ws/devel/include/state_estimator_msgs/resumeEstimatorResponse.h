// Generated by gencpp from file state_estimator_msgs/resumeEstimatorResponse.msg
// DO NOT EDIT!


#ifndef STATE_ESTIMATOR_MSGS_MESSAGE_RESUMEESTIMATORRESPONSE_H
#define STATE_ESTIMATOR_MSGS_MESSAGE_RESUMEESTIMATORRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace state_estimator_msgs
{
template <class ContainerAllocator>
struct resumeEstimatorResponse_
{
  typedef resumeEstimatorResponse_<ContainerAllocator> Type;

  resumeEstimatorResponse_()
    : success(false)  {
    }
  resumeEstimatorResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> const> ConstPtr;

}; // struct resumeEstimatorResponse_

typedef ::state_estimator_msgs::resumeEstimatorResponse_<std::allocator<void> > resumeEstimatorResponse;

typedef boost::shared_ptr< ::state_estimator_msgs::resumeEstimatorResponse > resumeEstimatorResponsePtr;
typedef boost::shared_ptr< ::state_estimator_msgs::resumeEstimatorResponse const> resumeEstimatorResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace state_estimator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimator_msgs/resumeEstimatorResponse";
  }

  static const char* value(const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct resumeEstimatorResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_estimator_msgs::resumeEstimatorResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MSGS_MESSAGE_RESUMEESTIMATORRESPONSE_H
