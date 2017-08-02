
#include "quad2eular.h"

#define ConstReg2Deg 57.3
//四元数转欧拉角
geometry_msgs::Vector3 quad2eular(geometry_msgs::Quaternion quad)
{
    geometry_msgs::Vector3 eular;

    eular.x = ConstReg2Deg * asin(-2 * quad.x * quad.z + 2 * quad.w * quad.y); // pitch
    eular.y = ConstReg2Deg * atan2(2 * quad.y * quad.z + 2 * quad.w * quad.x,
                                 -2 * quad.x * quad.x - 2 * quad.y * quad.y + 1); // roll
    eular.z = ConstReg2Deg * atan2(2 * (quad.x * quad.y + quad.w * quad.z),
                                 quad.w * quad.w + quad.x * quad.x - quad.y * quad.y - quad.z * quad.z); // yaw

    return eular;
}
