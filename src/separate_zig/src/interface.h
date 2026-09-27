#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer to hide C++ classes from the C compiler
typedef void* ROS2ContextHandle;

ROS2ContextHandle init_ros2(const char* node_name);
void shutdown_ros2(ROS2ContextHandle handle);

int call_add_two_ints_service(ROS2ContextHandle handle, const char* service_name, int a, int b, int* out_sum);

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_H

