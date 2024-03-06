// Generated by gencpp from file motoman_msgs/DynamicJointTrajectoryFeedback.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTTRAJECTORYFEEDBACK_H
#define MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTTRAJECTORYFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <motoman_msgs/DynamicJointState.h>

namespace motoman_msgs
{
template <class ContainerAllocator>
struct DynamicJointTrajectoryFeedback_
{
  typedef DynamicJointTrajectoryFeedback_<ContainerAllocator> Type;

  DynamicJointTrajectoryFeedback_()
    : header()
    , num_groups(0)
    , joint_feedbacks()  {
    }
  DynamicJointTrajectoryFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , num_groups(0)
    , joint_feedbacks(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int16_t _num_groups_type;
  _num_groups_type num_groups;

   typedef std::vector< ::motoman_msgs::DynamicJointState_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::motoman_msgs::DynamicJointState_<ContainerAllocator> >> _joint_feedbacks_type;
  _joint_feedbacks_type joint_feedbacks;





  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct DynamicJointTrajectoryFeedback_

typedef ::motoman_msgs::DynamicJointTrajectoryFeedback_<std::allocator<void> > DynamicJointTrajectoryFeedback;

typedef boost::shared_ptr< ::motoman_msgs::DynamicJointTrajectoryFeedback > DynamicJointTrajectoryFeedbackPtr;
typedef boost::shared_ptr< ::motoman_msgs::DynamicJointTrajectoryFeedback const> DynamicJointTrajectoryFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.num_groups == rhs.num_groups &&
    lhs.joint_feedbacks == rhs.joint_feedbacks;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "84d3bbf7103790ff0a8946017b895a1a";
  }

  static const char* value(const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x84d3bbf7103790ffULL;
  static const uint64_t static_value2 = 0x0a8946017b895a1aULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/DynamicJointTrajectoryFeedback";
  }

  static const char* value(const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int16 num_groups\n"
"DynamicJointState[] joint_feedbacks\n"
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
"\n"
"================================================================================\n"
"MSG: motoman_msgs/DynamicJointState\n"
"#group[]: # length of this array must match num_groups\n"
"#    id:   control-group ID for use on-controller\n"
"#    num_joints: # of joints in this motion group\n"
"#    valid_fields: #bit field for following items\n"
"#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n"
"#    positions[]\n"
"#    velocities[]\n"
"#    accelerations[]\n"
"#    effort[]\n"
"#    position_desired[]\n"
"#    position_errors[]\n"
"#    velocity_desired[]\n"
"#    velocity_errors[]\n"
"#    effort_desired[]\n"
"#    effort_error[]\n"
"\n"
"int16 group_number\n"
"int16 num_joints\n"
"int16 valid_fields\n"
"float64[] positions\n"
"float64[] velocities\n"
"float64[] accelerations\n"
"float64[] effort\n"
"float64[] positions_desired\n"
"float64[] positions_errors\n"
"float64[] velocities_desired\n"
"float64[] velocities_errors\n"
"float64[] accelerations_desired\n"
"float64[] accelerations_errors\n"
"float64[] effort_errors\n"
"float64[] effort_desired\n"
;
  }

  static const char* value(const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.num_groups);
      stream.next(m.joint_feedbacks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamicJointTrajectoryFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::DynamicJointTrajectoryFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "num_groups: ";
    Printer<int16_t>::stream(s, indent + "  ", v.num_groups);
    s << indent << "joint_feedbacks[]" << std::endl;
    for (size_t i = 0; i < v.joint_feedbacks.size(); ++i)
    {
      s << indent << "  joint_feedbacks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::motoman_msgs::DynamicJointState_<ContainerAllocator> >::stream(s, indent + "    ", v.joint_feedbacks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTTRAJECTORYFEEDBACK_H