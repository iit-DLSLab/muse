// Generated by gencpp from file state_estimator_msgs/getEstimatorDescriptionResponse.msg
// DO NOT EDIT!


#ifndef STATE_ESTIMATOR_MSGS_MESSAGE_GETESTIMATORDESCRIPTIONRESPONSE_H
#define STATE_ESTIMATOR_MSGS_MESSAGE_GETESTIMATORDESCRIPTIONRESPONSE_H


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
struct getEstimatorDescriptionResponse_
{
  typedef getEstimatorDescriptionResponse_<ContainerAllocator> Type;

  getEstimatorDescriptionResponse_()
    : success(false)
    , description()  {
    }
  getEstimatorDescriptionResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , description(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _description_type;
  _description_type description;





  typedef boost::shared_ptr< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct getEstimatorDescriptionResponse_

typedef ::state_estimator_msgs::getEstimatorDescriptionResponse_<std::allocator<void> > getEstimatorDescriptionResponse;

typedef boost::shared_ptr< ::state_estimator_msgs::getEstimatorDescriptionResponse > getEstimatorDescriptionResponsePtr;
typedef boost::shared_ptr< ::state_estimator_msgs::getEstimatorDescriptionResponse const> getEstimatorDescriptionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.description == rhs.description;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator1> & lhs, const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace state_estimator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6bccbb1e6ccd1459ac90cc79251ac541";
  }

  static const char* value(const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6bccbb1e6ccd1459ULL;
  static const uint64_t static_value2 = 0xac90cc79251ac541ULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimator_msgs/getEstimatorDescriptionResponse";
  }

  static const char* value(const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"string description\n"
"\n"
;
  }

  static const char* value(const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.description);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct getEstimatorDescriptionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_estimator_msgs::getEstimatorDescriptionResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.description);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MSGS_MESSAGE_GETESTIMATORDESCRIPTIONRESPONSE_H
