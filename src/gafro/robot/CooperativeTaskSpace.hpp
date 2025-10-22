#pragma once

#include <gafro/robot/TaskSpace.hpp>

namespace gafro
{
    template <class T, int points>
    struct CooperativePrimitive
    {
        using Type  = typename OuterProduct<Point<T>, CooperativePrimitive<T, points - 1>>::Type;
        using Vtype = T;

        static constexpr int  size   = Type::size;
        static constexpr auto blades = Type::blades;
    };

    template <class T>
    struct CooperativePrimitive<T, 1>
    {
        using Type  = typename Point<T>::Type;
        using Vtype = T;

        static constexpr int  size   = Type::size;
        static constexpr auto blades = Type::blades;
    };

    template <class T, int size, int dof>
    class CooperativeTaskSpace : public TaskSpace<T>
    {
      public:
        CooperativeTaskSpace(System<T> *system, const std::array<std::string, size> &kinematic_chains);

        virtual ~CooperativeTaskSpace();

        template <class S>
        using Type = typename CooperativePrimitive<S, size>::Type;

        using PrimitiveJacobian = typename Type<T>::template Matrix<1, dof>;

        //

        Eigen::VectorX<T> convertToSystemConfiguration(const Eigen::Vector<T, dof> &task_space_configuration) const;

        Eigen::VectorX<T> convertToTaskSpaceConfiguration(const Eigen::VectorX<T> &task_space_configuration) const;

        Eigen::VectorX<T> extractKinematicChainConfiguration(const std::string &name, const Eigen::VectorX<T> &task_space_configuration) const;

        ForwardKinematics<T> computeForwardKinematics(const Eigen::Vector<T, dof> &task_space_configuration) const;

        // size == 3

        // template <int dof>
        // Circle<T> computePrimitive(const Eigen::Vector<T, dof> &task_space_configuration) const

        // MultivectorMatrix<T, Circle, 1, dof> computeCircleJacobian(const Eigen::VectorX<T> &task_space_configuration) const;

        // size == 4
        typename Point<T>::template Matrix<1, size> computePoints(const Eigen::Vector<T, dof> &task_space_configuration) const;

        auto computePrimitive(const Eigen::Vector<T, dof> &task_space_configuration) const;

        PrimitiveJacobian computePrimitiveJacobian(const Eigen::Vector<T, dof> &task_space_configuration) const;

        // MultivectorMatrix<T, Sphere, 1, dof> computeSphereJacobian(const Eigen::VectorX<T> &task_space_configuration) const;

        SimilarityTransformation<T> computeSimilarityTransformation(const Eigen::Vector<T, dof> &task_space_configuration) const;

      protected:
      private:
        std::map<std::string, const KinematicChain<T> *> kinematic_chains_;

        std::map<std::string, std::vector<unsigned>> kinematic_chain_joint_indices_;

        std::map<unsigned, unsigned> joint_index_map_;
    };

}  // namespace gafro

#include <gafro/robot/CooperativeTaskSpace.hxx>