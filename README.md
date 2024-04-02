## 建立模型

```c++
  namespace pino=pinocchio;
  const std::string urdf_filename=PINOCCHIO_MODEL_DIR;
  pino::Model model;
  pino::urdf::buildModel(urdf_filename,model);

  // Create data required by the algorithms
  pino::Data data(model);

```

### Jointvarient

```c++
 typedef boost::variant<
//    JointModelVoid,
      JointModelRX, JointModelRY, JointModelRZ
    , JointModelMimicRX, JointModelMimicRY, JointModelMimicRZ
    , JointModelFreeFlyer, JointModelPlanar
    , JointModelRevoluteUnaligned
    , JointModelSpherical, JointModelSphericalZYX
    , JointModelPX, JointModelPY, JointModelPZ
    , JointModelPrismaticUnaligned
    , JointModelTranslation
    , JointModelRUBX, JointModelRUBY, JointModelRUBZ
    , JointModelRevoluteUnboundedUnaligned
    , boost::recursive_wrapper<JointModelComposite>
    > JointModelVariant;
```

Boost.Variant 提供了一个类似于 union 的名为 boost::variant 的类。您可以将不同类型的值存储在 boost::variant 变量中。在任何时候只能存储一个值。分配新值时，旧值将被覆盖。但是，新值的类型可能与旧值不同。唯一的要求是这些类型必须作为模板参数传递给 boost::variant，这样它们才能为 boost::variant 变量所知。

```c++
#include <boost/variant.hpp>
#include <string>
 
int main()
{
  boost::variant<double, char, std::string> v;
  v = 3.14;
  v = 'A';
  v = "Boost";
}
```

### 浮动基

```c++
q = [global_base_position, global_base_quaternion, joint_positions]
v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]

```

The base translation part is expressed in the parent frame (here the world coordinate system) while its velocity is expressed in the body coordinate system.

### rpy转四元数方法

```c++
Eigen::Matrix3d rotationMatrix = pinocchio::rpyToMatrix(roll, pitch, yaw);

    Eigen::Quaterniond quaternion(rotationMatrix);
```



## SE（3）

$$
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}\in R^{4\times4}
$$

表示方向,两个frame的相对orientation,B在A中表示:
$$
^AR_B=\begin{bmatrix}^A\hat{x}_B &^A\hat{y}_B&^A\hat{z}_B\end{bmatrix}
$$

```c++
 data.oMi[joint_id];
 data.oMi[joint_id].translation()
 data.oMi[joint_id].rotation()
```



## SO(3)

$$
so(3)={R\in R^{3\times3}:R^TR=I,det(R)=1}
$$



表示变换:
$$
^AC_B=^AR_B
$$
![image-20240313112914569](C:\Users\xsy\AppData\Roaming\Typora\typora-user-images\image-20240313112914569.png)

## Frame坐标转换

![image-20240317142126591](C:\Users\xsy\AppData\Roaming\Typora\typora-user-images\image-20240317142126591.png)
$$
m\in M^6,f\in F^6
\newline
m_B=^BX_Am_A,f_B=^BX_A^Ff_A
$$
Frame：universe,world,root_joint,以及urdf中所有joint和link的坐标

## ComputeAllterms

包含如下计算：

### pinocchio::forwardKinematics

* Update the joint placements，spatial velocities ,spatial accelerations according to the current joint configuration,velocity,acceleration。

### pinocchio::crba

* joint-space formulation:
  $$
  H(q)\ddot{q}+C(q,\dot{q})\dot{q}+\tau_g(q)=\tau
  $$

  $$
  \begin{bmatrix}
  \tau_f  \\
  \tau_j \\
  \end{bmatrix}=H(q)\ddot{q}+C(q,\dot{q})+g-J_c^Tf_r
  $$

  计算JSIM上三角部分,下三角部分:

  ```c++
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ```

### pinocchio::nonLinearEffects

* Calculate Corriolis, centrifual and gravitationnal effects
* pinocchio::rnea(model,data,q,v,0)

### pinocchio::computeJointJacobians



### pinocchio::centerOfMass()

* return The center of mass position of the full rigid body system expressed in the world frame.

* Parameters

  |      | model              | The model structure of the rigid body system.                |
  | ---- | ------------------ | ------------------------------------------------------------ |
  |      | data               | The data structure of the rigid body system.                 |
  |      | q                  | The joint configuration vector (dim model.nq).               |
  | [    | computeSubtreeComs | If true, the algorithm computes also the center of mass of the subtrees. |

### pinocchio::ccrba

* centroidal composit-rigid-body algorithm

* 返回质心动量矩阵$A_G$
  $$
  h_G=A_G(q)\dot{q}
  $$



##  pinocchio::UpdataFramePlacement



* 更新各个Frame的位置(pino::forwardkinematics更新各个joint的位置)

## Pinocchio::Datatpl

* data.M. The joint space inertia matrix (a square matrix of dim model.nv).

* data.nle.In the multibody dynamics equation $M\ddot{q}+b(q,\dot{q})=\tau$, term $b$

  

## Pinocchio::Frametpl

* placement(SE3):Placement of the frame wrt the parent joint.
