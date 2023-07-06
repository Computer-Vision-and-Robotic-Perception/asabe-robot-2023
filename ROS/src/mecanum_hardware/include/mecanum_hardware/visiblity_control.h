/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef MECANUM_HARDWARE__VISIBLITY_CONTROL_H_
#define MECANUM_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MECANUM_HARDWARE_EXPORT __attribute__((dllexport))
#define MECANUM_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MECANUM_HARDWARE_EXPORT __declspec(dllexport)
#define MECANUM_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MECANUM_HARDWARE_BUILDING_DLL
#define MECANUM_HARDWARE_PUBLIC MECANUM_HARDWARE_EXPORT
#else
#define MECANUM_HARDWARE_PUBLIC MECANUM_HARDWARE_IMPORT
#endif
#define MECANUM_HARDWARE_PUBLIC_TYPE MECANUM_HARDWARE_PUBLIC
#define MECANUM_HARDWARE_LOCAL
#else
#define MECANUM_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MECANUM_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MECANUM_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MECANUM_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MECANUM_HARDWARE_PUBLIC
#define MECANUM_HARDWARE_LOCAL
#endif
#define MECANUM_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MECANUM_HARDWARE__VISIBLITY_CONTROL_H_
