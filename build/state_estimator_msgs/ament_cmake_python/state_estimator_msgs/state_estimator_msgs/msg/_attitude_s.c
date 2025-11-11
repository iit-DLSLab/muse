// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "state_estimator_msgs/msg/detail/attitude__struct.h"
#include "state_estimator_msgs/msg/detail/attitude__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool state_estimator_msgs__msg__attitude__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
    if (class_attr == NULL) {
      return false;
    }
    PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
    if (name_attr == NULL) {
      Py_DECREF(class_attr);
      return false;
    }
    PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
    if (module_attr == NULL) {
      Py_DECREF(name_attr);
      Py_DECREF(class_attr);
      return false;
    }

    // PyUnicode_1BYTE_DATA is just a cast
    assert(strncmp("state_estimator_msgs.msg._attitude", (char *)PyUnicode_1BYTE_DATA(module_attr), 34) == 0);
    assert(strncmp("Attitude", (char *)PyUnicode_1BYTE_DATA(name_attr), 8) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  state_estimator_msgs__msg__Attitude * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // quaternion
    PyObject * field = PyObject_GetAttrString(_pymsg, "quaternion");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 4;
      double * dest = ros_message->quaternion;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // roll_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_deg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_deg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_deg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 3;
      double * dest = ros_message->angular_velocity;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * state_estimator_msgs__msg__attitude__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Attitude */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("state_estimator_msgs.msg._attitude");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Attitude");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  state_estimator_msgs__msg__Attitude * ros_message = (state_estimator_msgs__msg__Attitude *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quaternion
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "quaternion");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->quaternion[0]);
    memcpy(dst, src, 4 * sizeof(double));
    Py_DECREF(field);
  }
  {  // roll_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_velocity
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "angular_velocity");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->angular_velocity[0]);
    memcpy(dst, src, 3 * sizeof(double));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
