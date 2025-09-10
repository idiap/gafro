#pragma once

#include <gafro/robot/TaskSpace.hpp>

namespace gafro
{

    template <class T, int size, int dof>
    class CooperativeTaskSpace : public TaskSpace<T>
    {
      public:
        CooperativeTaskSpace(System<T> *system, const std::array<std::string, size> &kinematic_chains);

        virtual ~CooperativeTaskSpace();

        //

        Eigen::VectorX<T> convertToSystemConfiguration(const Eigen::Vector<T, dof> &task_space_configuration) const;

        Eigen::VectorX<T> convertToTaskSpaceConfiguration(const Eigen::VectorX<T> &task_space_configuration) const;

        // size == 3

        // template <int dof>
        // Circle<T> computePrimitive(const Eigen::Vector<T, dof> &task_space_configuration) const

        // MultivectorMatrix<T, Circle, 1, dof> computeCircleJacobian(const Eigen::VectorX<T> &task_space_configuration) const;

        // size == 4

        auto computePrimitive(const Eigen::Vector<T, dof> &task_space_configuration) const;

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