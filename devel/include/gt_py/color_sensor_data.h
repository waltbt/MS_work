// Generated by gencpp from file gt_py/color_sensor_data.msg
// DO NOT EDIT!


#ifndef GT_PY_MESSAGE_COLOR_SENSOR_DATA_H
#define GT_PY_MESSAGE_COLOR_SENSOR_DATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gt_py
{
template <class ContainerAllocator>
struct color_sensor_data_
{
  typedef color_sensor_data_<ContainerAllocator> Type;

  color_sensor_data_()
    : rgb_values()
    , color_temp(0)
    , lux(0.0)  {
      rgb_values.assign(0);
  }
  color_sensor_data_(const ContainerAllocator& _alloc)
    : rgb_values()
    , color_temp(0)
    , lux(0.0)  {
  (void)_alloc;
      rgb_values.assign(0);
  }



   typedef boost::array<int32_t, 4>  _rgb_values_type;
  _rgb_values_type rgb_values;

   typedef int32_t _color_temp_type;
  _color_temp_type color_temp;

   typedef float _lux_type;
  _lux_type lux;





  typedef boost::shared_ptr< ::gt_py::color_sensor_data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gt_py::color_sensor_data_<ContainerAllocator> const> ConstPtr;

}; // struct color_sensor_data_

typedef ::gt_py::color_sensor_data_<std::allocator<void> > color_sensor_data;

typedef boost::shared_ptr< ::gt_py::color_sensor_data > color_sensor_dataPtr;
typedef boost::shared_ptr< ::gt_py::color_sensor_data const> color_sensor_dataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gt_py::color_sensor_data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gt_py::color_sensor_data_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gt_py

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'gt_py': ['/home/ben/gripper_proj_py/src/gt_py/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gt_py::color_sensor_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gt_py::color_sensor_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gt_py::color_sensor_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gt_py::color_sensor_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gt_py::color_sensor_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gt_py::color_sensor_data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gt_py::color_sensor_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7c8cfe28df9aa44956a941b8c260653a";
  }

  static const char* value(const ::gt_py::color_sensor_data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7c8cfe28df9aa449ULL;
  static const uint64_t static_value2 = 0x56a941b8c260653aULL;
};

template<class ContainerAllocator>
struct DataType< ::gt_py::color_sensor_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gt_py/color_sensor_data";
  }

  static const char* value(const ::gt_py::color_sensor_data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gt_py::color_sensor_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[4] rgb_values\n\
int32 color_temp\n\
float32 lux\n\
";
  }

  static const char* value(const ::gt_py::color_sensor_data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gt_py::color_sensor_data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rgb_values);
      stream.next(m.color_temp);
      stream.next(m.lux);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct color_sensor_data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gt_py::color_sensor_data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gt_py::color_sensor_data_<ContainerAllocator>& v)
  {
    s << indent << "rgb_values[]" << std::endl;
    for (size_t i = 0; i < v.rgb_values.size(); ++i)
    {
      s << indent << "  rgb_values[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.rgb_values[i]);
    }
    s << indent << "color_temp: ";
    Printer<int32_t>::stream(s, indent + "  ", v.color_temp);
    s << indent << "lux: ";
    Printer<float>::stream(s, indent + "  ", v.lux);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GT_PY_MESSAGE_COLOR_SENSOR_DATA_H
