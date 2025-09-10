#include <gafro/robot/CooperativeTaskSpace.hpp>

namespace gafro
{
    template <class T, int size, int dof>
    CooperativeTaskSpace<T, size, dof>::CooperativeTaskSpace(System<T> *system, const std::array<std::string, size> &kinematic_chains)
      : TaskSpace<T>(system)
    {
        std::map<std::string, unsigned> actuated_joints;

        for (const auto &kc : kinematic_chains)
        {
            if (!this->getSystem()->hasKinematicChain(kc))
            {
                this->getSystem()->createKinematicChain(kc);
            }

            kinematic_chains_.emplace(std::make_pair(kc, this->getSystem()->getKinematicChain(kc)));

            std::vector<unsigned> joint_indices;

            for (const auto &joint : this->getSystem()->getKinematicChain(kc)->getActuatedJoints())
            {
                if (actuated_joints.find(joint->getName()) == actuated_joints.end())
                {
                    actuated_joints.emplace(std::make_pair(joint->getName(), actuated_joints.size()));
                }

                joint_index_map_.emplace(std::make_pair(actuated_joints.size() - 1, joint->getIndex()));

                joint_indices.push_back(actuated_joints[joint->getName()]);
            }

            kinematic_chain_joint_indices_.emplace(std::make_pair(kc, joint_indices));
        }

        assert(dof == actuated_joints.size());
    }

    template <class T, int size, int dof>
    CooperativeTaskSpace<T, size, dof>::~CooperativeTaskSpace() = default;

    template <class T, int size, int dof>
    Eigen::VectorX<T> CooperativeTaskSpace<T, size, dof>::convertToSystemConfiguration(const Eigen::Vector<T, dof> &task_space_configuration) const
    {
        Eigen::VectorX<T> system_configuration = Eigen::VectorX<T>::Zero(this->getSystem()->getDoF());

        for (const auto &index_pair : joint_index_map_)
        {
            system_configuration[index_pair.second] = task_space_configuration[index_pair.first];
        }

        return system_configuration;
    }

    template <class T, int size, int dof>
    Eigen::VectorX<T> CooperativeTaskSpace<T, size, dof>::convertToTaskSpaceConfiguration(const Eigen::VectorX<T> &system_configuration) const
    {
        assert(system_configuration.rows() == this->getSystem()->getDoF());

        Eigen::VectorX<T> task_space_configuration = Eigen::Vector<T, dof>::Zero();

        for (const auto &index_pair : joint_index_map_)
        {
            task_space_configuration[index_pair.first] = system_configuration[index_pair.second];
        }

        return task_space_configuration;
    }

    template <class T, int size, int dof>
    auto CooperativeTaskSpace<T, size, dof>::computePrimitive(const Eigen::Vector<T, dof> &task_space_configuration) const
    {
        Eigen::VectorX<T> system_configuration = convertToSystemConfiguration(task_space_configuration);

        auto forward_kinematics = this->getSystem()->computeForwardKinematics(system_configuration);

        std::array<Point<T>, size> points;

        int i = 0;

        for (const auto &indices : kinematic_chain_joint_indices_)
        {
            const auto *kinematic_chain = this->getSystem()->getKinematicChain(indices.first);

            points[i++] = forward_kinematics.getJointPose(kinematic_chain->getName()).apply(Point<T>());
        }

        if constexpr (size == 3)
        {
            return Circle<T>(points[0], points[1], points[2]);
        }
        else if constexpr (size == 4)
        {
            return Sphere<T>(points[0], points[1], points[2], points[3]);
        }
    }

    template <class T, int size, int dof>
    SimilarityTransformation<T> CooperativeTaskSpace<T, size, dof>::computeSimilarityTransformation(
      const Eigen::Vector<T, dof> &task_space_configuration) const
    {
        if constexpr (size == 3)
        {
            return Circle<T>::Unit().getSimilarityTransformation(computePrimitive(task_space_configuration));
        }
        else if constexpr (size == 4)
        {
            return Sphere<T>::Unit().getSimilarityTransformation(computePrimitive(task_space_configuration));
        }

        return SimilarityTransformation<T>();
    }

}  // namespace gafro