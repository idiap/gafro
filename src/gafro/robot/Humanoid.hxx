#include <gafro/robot/Humanoid.hpp>

namespace gafro
{

    template <class T>
    Humanoid<T>::Humanoid(System<T> &&humanoid)
    {
        *this = std::move(humanoid);
    }

    template <class T>
    Humanoid<T> &Humanoid<T>::operator=(System<T> &&humanoid)
    {
        this->System<T>::operator=(std::move(humanoid));

        return *this;
    }

    template <class T>
    Humanoid<T>::Humanoid(Humanoid &&humanoid)
    {
        *this = std::move(humanoid);
    }

    template <class T>
    Humanoid<T> &Humanoid<T>::operator=(Humanoid &&humanoid)
    {
        this->System<T>::operator=(std::move(humanoid));

        return *this;
    }

    template <class T>
    Humanoid<T>::~Humanoid() = default;

}  // namespace gafro