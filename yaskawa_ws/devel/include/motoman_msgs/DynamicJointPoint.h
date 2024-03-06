// Generated by gencpp from file motoman_msgs/DynamicJointPoint.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTPOINT_H
#define MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTPOINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <motoman_msgs/DynamicJointsGroup.h>

namespace motoman_msgs
{
template <class ContainerAllocator>
struct DynamicJointPoint_
{
  typedef DynamicJointPoint_<ContainerAllocator> Type;

  DynamicJointPoint_()
    : num_groups(0)
    , groups()  {
    }
  DynamicJointPoint_(const ContainerAllocator& _alloc)
    : num_groups(0)
    , groups(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _num_groups_type;
  _num_groups_type num_groups;

   typedef std::vector< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >> _groups_type;
  _groups_type groups;





  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> const> ConstPtr;

}; // struct DynamicJointPoint_

typedef ::motoman_msgs::DynamicJointPoint_<std::allocator<void> > DynamicJointPoint;

typedef boost::shared_ptr< ::motoman_msgs::DynamicJointPoint > DynamicJointPointPtr;
typedef boost::shared_ptr< ::motoman_msgs::DynamicJointPoint const> DynamicJointPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator2> & rhs)
{
  return lhs.num_groups == rhs.num_groups &&
    lhs.groups == rhs.groups;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f91ca86c2821b55c8430ab0088bfe5df";
  }

  static const char* value(const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf91ca86c2821b55cULL;
  static const uint64_t static_value2 = 0x8430ab0088bfe5dfULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/DynamicJointPoint";
  }

  static const char* value(const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# DynamicJointPoint\n"
"#group: # length of this array must match num_groups\n"
"#    id:   control-group ID for use on-controller\n"
"#    num_joints: # of joints in this motion group\n"
"#    valid_fields: #bit field for following items\n"
"#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n"
"#    positions[]\n"
"#    velocities[]\n"
"#    accelerations[]\n"
"#    effort[]\n"
"#    time_from_start\n"
"\n"
"int16 num_groups\n"
"DynamicJointsGroup[] groups\n"
"\n"
"================================================================================\n"
"MSG: motoman_msgs/DynamicJointsGroup\n"
"# DynamicJointsGroup\n"
"#group: # length of this array must match num_groups\n"
"#    id:   control-group ID for use on-controller\n"
"#    num_joints: # of joints in this motion group\n"
"#    valid_fields: #bit field for following items\n"
"#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n"
"#    positions[]\n"
"#    velocities[]\n"
"#    accelerations[]\n"
"#    effort[]\n"
"#    time_from_start\n"
"\n"
"\n"
"int16 group_number\n"
"int16 num_joints\n"
"int16 valid_fields\n"
"float64[] positions\n"
"float64[] velocities\n"
"float64[] accelerations\n"
"float64[] effort\n"
"duration time_from_start\n"
;
  }

  static const char* value(const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num_groups);
      stream.next(m.groups);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamicJointPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::DynamicJointPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::DynamicJointPoint_<ContainerAllocator>& v)
  {
    s << indent << "num_groups: ";
    Printer<int16_t>::stream(s, indent + "  ", v.num_groups);
    s << indent << "groups[]" << std::endl;
    for (size_t i = 0; i < v.groups.size(); ++i)
    {
      s << indent << "  groups[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >::stream(s, indent + "    ", v.groups[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTPOINT_H
