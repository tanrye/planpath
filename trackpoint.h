// Generated by gencpp from file CameraMsg/TrackPoint.msg
// DO NOT EDIT!


#ifndef CAMERAMSG_MESSAGE_TRACKPOINT_H
#define CAMERAMSG_MESSAGE_TRACKPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace CameraMsg
{
template <class ContainerAllocator>
struct TrackPoint_
{
  typedef TrackPoint_<ContainerAllocator> Type;

  TrackPoint_()
    : x()
    , y()  {
    }
  TrackPoint_(const ContainerAllocator& _alloc)
    : x(_alloc)
    , y(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _x_type;
  _x_type x;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::CameraMsg::TrackPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CameraMsg::TrackPoint_<ContainerAllocator> const> ConstPtr;

}; // struct TrackPoint_

typedef ::CameraMsg::TrackPoint_<std::allocator<void> > TrackPoint;

typedef boost::shared_ptr< ::CameraMsg::TrackPoint > TrackPointPtr;
typedef boost::shared_ptr< ::CameraMsg::TrackPoint const> TrackPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::CameraMsg::TrackPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::CameraMsg::TrackPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace CameraMsg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'CameraMsg': ['/home/wang/workspace/RoboWare_workspace/ros_messages/src/CameraMsg/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::CameraMsg::TrackPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::CameraMsg::TrackPoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::CameraMsg::TrackPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::CameraMsg::TrackPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::CameraMsg::TrackPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::CameraMsg::TrackPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::CameraMsg::TrackPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d02d5b87dd0a8706354704c31a3c30e";
  }

  static const char* value(const ::CameraMsg::TrackPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d02d5b87dd0a870ULL;
  static const uint64_t static_value2 = 0x6354704c31a3c30eULL;
};

template<class ContainerAllocator>
struct DataType< ::CameraMsg::TrackPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "CameraMsg/TrackPoint";
  }

  static const char* value(const ::CameraMsg::TrackPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::CameraMsg::TrackPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] x\n\
int32[] y\n\
";
  }

  static const char* value(const ::CameraMsg::TrackPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::CameraMsg::TrackPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::CameraMsg::TrackPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::CameraMsg::TrackPoint_<ContainerAllocator>& v)
  {
    s << indent << "x[]" << std::endl;
    for (size_t i = 0; i < v.x.size(); ++i)
    {
      s << indent << "  x[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "y[]" << std::endl;
    for (size_t i = 0; i < v.y.size(); ++i)
    {
      s << indent << "  y[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.y[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAMERAMSG_MESSAGE_TRACKPOINT_H
