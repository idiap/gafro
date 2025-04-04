#pragma once

#include <gafro/gafro.hpp>

namespace gafro::probabilistic
{

    template <class T, template <class S> class Multivector>
    class MultivectorGaussian
    {
      public:
        using Mean = typename Multivector<T>::Type;
        constexpr static int size = Mean::size;
        using Covariance = Eigen::Matrix<T, size, size>;

        MultivectorGaussian();

        MultivectorGaussian(const Mean &primitive, const Covariance &covariance);

        virtual ~MultivectorGaussian();

        void setMean(const Mean &mean);

        void setCovariance(const Covariance &covariance);

        Mean sample() const;

        const Mean &getMean() const;

        const Covariance &getCovariance() const;

        template <class Derived, int... blades>
        MultivectorGaussian transform(const typename Versor<Derived, T, blades...>::Gaussian &versor) const;

        T getProbability(const gafro::Point<T> &point) const;

        T getProbability(const gafro::Vector<T> &vector) const;

        T getDistance(const gafro::Point<T> &point) const;

        T getDistance(const gafro::Vector<T> &vector) const;

        T getLikelihood(const Mean &sample) const;

        gafro::Point<T> getGradient(const gafro::Point<T> &point) const;

        gafro::Vector<T> getGradient(const gafro::Vector<T> &vector) const;

        Eigen::Matrix<T, 5, 5> getHessian(const gafro::Point<T> &point) const;

        template <template <class S> class Other>
        auto operator|(const MultivectorGaussian<T, Other> &other) const;

        template <template <class S> class Other>
            requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
        auto operator|(const Other<T> &other) const;

        template <template <class S> class Other>
        auto operator^(const MultivectorGaussian<T, Other> &other) const;

        template <template <class S> class Other>
            requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
        auto operator^(const Other<T> &other) const;

        template <template <class S> class Other>
        auto operator*(const MultivectorGaussian<T, Other> &other) const;

        template <template <class S> class Other>
            requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
        auto operator*(const Other<T> &other) const;

      protected:
      private:
        Mean mean_;

        Covariance covariance_;

      public:
        static MultivectorGaussian Zero();
    };

    // template <class T, template <class> class Multivector, template <class> class Other>
    // auto operator*(const MultivectorGaussian<T, Multivector> &gaussian, const MultivectorGaussian<T, Other> &other);

}  // namespace gafro::probabilistic

#include <gafro/probabilistic/MultivectorGaussian.hxx>