// Generated by gencpp from file plan_path/lane_point.msg
// DO NOT EDIT!


#ifndef PLAN_PATH_MESSAGE_LANE_POINT_H
#define PLAN_PATH_MESSAGE_LANE_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace plan_path
{
template <class ContainerAllocator>
struct lane_point_
{
  typedef lane_point_<ContainerAllocator> Type;

  lane_point_()
    : x(0)
    , y(0)  {
    }
  lane_point_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)  {
  (void)_alloc;
    }



   typedef int32_t _x_type;
  _x_type x;

   typedef int32_t _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::plan_path::lane_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plan_path::lane_point_<ContainerAllocator> const> ConstPtr;

}; // struct lane_point_

typedef ::plan_path::lane_point_<std::allocator<void> > lane_point;

typedef boost::shared_ptr< ::plan_path::lane_point > lane_pointPtr;
typedef boost::shared_ptr< ::plan_path::lane_point const> lane_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plan_path::lane_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plan_path::lane_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace plan_path

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'plan_path': ['/home/wang/catkin_paththird/src/plan_path/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::plan_path::lane_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_path::lane_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_path::lane_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_path::lane_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_path::lane_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_path::lane_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plan_path::lane_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd7b43fd41d4c47bf5c703cc7d016709";
  }

  static const char* value(const ::plan_path::lane_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd7b43fd41d4c47bULL;
  static const uint64_t static_value2 = 0xf5c703cc7d016709ULL;
};

template<class ContainerAllocator>
struct DataType< ::plan_path::lane_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plan_path/lane_point";
  }

  static const char* value(const ::plan_path::lane_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plan_path::lane_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 x\n\
int32 y\n\
";
  }

  static const char* value(const ::plan_path::lane_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plan_path::lane_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct lane_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plan_path::lane_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plan_path::lane_point_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLAN_PATH_MESSAGE_LANE_POINT_H
