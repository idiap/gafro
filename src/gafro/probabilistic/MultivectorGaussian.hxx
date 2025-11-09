#pragma once

#include <gafro/probabilistic/MultivectorGaussian.hpp>
#include <random>

namespace gafro::probabilistic
{

    template <class T, template <class S> class Multivector>
    MultivectorGaussian<T, Multivector>::MultivectorGaussian()
    {}

    template <class T, template <class S> class Multivector>
    MultivectorGaussian<T, Multivector>::MultivectorGaussian(const Mean &primitive, const Covariance &covariance)
      : Mean(primitive), covariance_(covariance)
    {}

    template <class T, template <class S> class Multivector>
    MultivectorGaussian<T, Multivector>::~MultivectorGaussian() = default;

    template <class T, template <class S> class Multivector>
    void MultivectorGaussian<T, Multivector>::setMean(const Mean &mean)
    {
        mean_ = mean;
    }

    template <class T, template <class S> class Multivector>
    void MultivectorGaussian<T, Multivector>::setCovariance(const Covariance &covariance)
    {
        covariance_ = covariance;
    }

    template <class T, template <class S> class Multivector>
    typename MultivectorGaussian<T, Multivector>::Mean MultivectorGaussian<T, Multivector>::sample() const
    {
        static std::default_random_engine generator;
        static std::normal_distribution<double> distribution;
        static auto random = [&]() { return distribution(generator); };

        return typename Mean::Parameters(mean_.vector() + covariance_.llt().matrixL() * Mean::Parameters::NullaryExpr(random));
    }

    template <class T, template <class S> class Multivector>
    const typename MultivectorGaussian<T, Multivector>::Mean &MultivectorGaussian<T, Multivector>::getMean() const
    {
        return mean_;
    }

    template <class T, template <class S> class Multivector>
    const typename MultivectorGaussian<T, Multivector>::Covariance &MultivectorGaussian<T, Multivector>::getCovariance() const
    {
        return covariance_;
    }

    template <class T, template <class S> class Multivector>
    template <class Derived, int... blades>
    MultivectorGaussian<T, Multivector> MultivectorGaussian<T, Multivector>::transform(
      const typename Versor<Derived, T, blades...>::Gaussian &versor) const
    {
        MultivectorGaussian<T, Multivector> transformed;

        transformed.setMean(versor.getMean().apply(this->getMean()));

        auto e1 = versor.getMean() * this->getMean();
        auto e2 = e1.evaluate() * versor.getMean().reverse().evaluate();

        Eigen::MatrixXd reverse = Versor<Derived, T, blades...>::One().reverse().vector().asDiagonal();

        Eigen::MatrixXd jl1 = e1.getLeftJacobian();
        Eigen::MatrixXd jr1 = e1.getRightJacobian();
        Eigen::MatrixXd jl2 = e2.getLeftJacobian();
        Eigen::MatrixXd jr2 = e2.getRightJacobian();

        Eigen::MatrixXd a = jl2 * jl1;
        Eigen::MatrixXd b = jr2 * reverse;

        Eigen::MatrixXd covariance = a * versor.getCovariance() * a.transpose() + b * versor.getCovariance() * b.transpose() +
                                     a * versor.getCovariance() * b.transpose() + b * versor.getCovariance() * a.transpose();

        return transformed;
    }

    template <class T, template <class S> class Multivector>
    MultivectorGaussian<T, Multivector> MultivectorGaussian<T, Multivector>::Zero()
    {
        return MultivectorGaussian(Mean({ 1.0 }), Covariance::Zero());
    }

    template <class T, template <class S> class Multivector>
    T MultivectorGaussian<T, Multivector>::getProbability(const gafro::Point<T> &point) const
    {
        using Result = typename gafro::OuterProduct<Mean, gafro::Point<T>>::Type;
        constexpr static int size = Result::size;

        auto expression = this->getMean() ^ point;
        auto left_jacobian = expression.getLeftJacobian();

        Eigen::Matrix<T, size, size> covariance =
          left_jacobian * covariance_ * left_jacobian.transpose() + 1e-7 * Eigen::Matrix<T, size, size>::Identity();

        T probability = (expression.vector().transpose() * covariance.inverse() * expression.vector()).value();

        return std::exp(-0.5 * probability);
    }

    template <class T, template <class S> class Multivector>
    T MultivectorGaussian<T, Multivector>::getProbability(const gafro::Vector<T> &vector) const
    {
        gafro::Point<T> point(vector.template get<gafro::blades::e1>(), vector.template get<gafro::blades::e2>(),
                              vector.template get<gafro::blades::e3>());

        return getProbability(point);
    }

    template <class T, template <class S> class Multivector>
    T MultivectorGaussian<T, Multivector>::getDistance(const gafro::Point<T> &point) const
    {
        using Result = typename gafro::OuterProduct<Mean, gafro::Point<T>>::Type;
        constexpr static int size = Result::size;

        auto expression = this->getMean() ^ point;
        auto left_jacobian = expression.getLeftJacobian();

        Eigen::Matrix<T, size, size> covariance =
          left_jacobian * covariance_ * left_jacobian.transpose() + 1e-7 * Eigen::Matrix<T, size, size>::Identity();

        return (expression.vector().transpose() * covariance.inverse() * expression.vector()).value();
    }

    template <class T, template <class S> class Multivector>
    T MultivectorGaussian<T, Multivector>::getDistance(const gafro::Vector<T> &vector) const
    {
        gafro::Point<T> point(vector.template get<gafro::blades::e1>(), vector.template get<gafro::blades::e2>(),
                              vector.template get<gafro::blades::e3>());

        return getDistance(point);
    }

    template <class T, template <class S> class Multivector>
    T MultivectorGaussian<T, Multivector>::getLikelihood(const Mean &sample) const
    {
        Eigen::Vector<T, size> difference = (this->getMean() - sample).vector();
        Eigen::Matrix<T, size, size> covariance = this->getCovariance() + Eigen::Matrix<T, size, size>::Identity();

        return std::exp(-0.5 * (difference.transpose() * covariance.inverse() * difference).value());
    }

    template <class T, template <class S> class Multivector>
    gafro::Point<T> MultivectorGaussian<T, Multivector>::getGradient(const gafro::Point<T> &point) const
    {
        using Result = typename gafro::OuterProduct<Mean, gafro::Point<T>>::Type;
        constexpr static int rsize = Result::size;

        auto expr = this->getMean() ^ point;

        auto tensor = expr.getTensor();
        auto left_jacobian = expr.getLeftJacobian();
        auto right_jacobian = expr.getRightJacobian();

        Eigen::Matrix<T, rsize, rsize> metric =
          (left_jacobian * covariance_ * left_jacobian.transpose() + 1e-7 * Eigen::Matrix<T, rsize, rsize>::Identity()).inverse();

        Eigen::Matrix<T, 5, 1> gradient = Eigen::Matrix<T, 5, 1>::Zero();

        for (unsigned k = 0; k < 5; ++k)
        {
            Eigen::Matrix<T, rsize, size> left_hessian = Eigen::Matrix<T, rsize, size>::Zero();

            for (unsigned j = 0; j < rsize; ++j)
            {
                left_hessian.row(j) = tensor[j].col(k).transpose();
            }

            Eigen::Matrix<T, rsize, rsize> metric_gradient =
              left_hessian * covariance_ * left_jacobian.transpose() + left_jacobian * covariance_ * left_hessian.transpose();

            gradient[k] = -point.vector().transpose() * right_jacobian.transpose() * metric.transpose() * metric_gradient * metric * right_jacobian *
                          point.vector();
        }

        gradient += 2.0 * right_jacobian.transpose() * metric * right_jacobian * point.vector();

        return gradient;
    }

    template <class T, template <class S> class Multivector>
    gafro::Vector<T> MultivectorGaussian<T, Multivector>::getGradient(const gafro::Vector<T> &vector) const
    {
        gafro::Point<T> point(vector.template get<gafro::blades::e1>(),  //
                              vector.template get<gafro::blades::e2>(),  //
                              vector.template get<gafro::blades::e3>());

        Eigen::Matrix<T, 5, 3> jacobian = point.getEmbeddingJacobian();

        return Eigen::Vector3d(jacobian.transpose() * getGradient(point).vector());
    }

    template <class T, template <class S> class Multivector>
    Eigen::Matrix<T, 5, 5> MultivectorGaussian<T, Multivector>::getHessian(const gafro::Point<T> &point) const
    {
        using Result = typename gafro::OuterProduct<Mean, gafro::Point<T>>::Type;
        constexpr static int rsize = Result::size;

        auto expr = this->getMean() ^ point;

        auto tensor = expr.getTensor();
        auto left_jacobian = expr.getLeftJacobian();
        auto right_jacobian = expr.getRightJacobian();

        Eigen::Matrix<T, rsize, rsize> metric =
          (left_jacobian * this->getCovariance() * left_jacobian.transpose() + 1e-7 * Eigen::Matrix<T, rsize, rsize>::Identity()).inverse();

        Eigen::Matrix<T, 5, 5> hessian = Eigen::Matrix<T, 5, 5>::Zero();

        std::vector<Eigen::Matrix<T, rsize, size>> left_hessians;
        std::vector<Eigen::Matrix<T, rsize, rsize>> metric_gradients;

        for (unsigned k = 0; k < 5; ++k)
        {
            Eigen::Matrix<T, rsize, size> left_hessian = Eigen::Matrix<T, rsize, size>::Zero();

            for (unsigned j = 0; j < rsize; ++j)
            {
                left_hessian.row(j) = tensor[j].col(k).transpose();
            }

            Eigen::Matrix<T, rsize, rsize> metric_gradient =
              left_hessian * this->getCovariance() * left_jacobian.transpose() + left_jacobian * this->getCovariance() * left_hessian.transpose();

            left_hessians.push_back(left_hessian);
            metric_gradients.push_back(metric_gradient);
        }

        for (unsigned k1 = 0; k1 < 5; ++k1)
        {
            for (unsigned k2 = 0; k2 <= k1; ++k2)
            {
                Eigen::Matrix<T, rsize, rsize> d1 = -metric_gradients[k2] * metric * metric_gradients[k1];

                Eigen::Matrix<T, rsize, rsize> d2 = (left_hessians[k1] * this->getCovariance() * left_hessians[k2].transpose() +
                                                     left_hessians[k2] * this->getCovariance() * left_hessians[k1].transpose());

                Eigen::Matrix<T, rsize, rsize> d3 = -metric_gradients[k1] * metric * metric_gradients[k2];

                Eigen::Matrix<T, rsize, rsize> derivative = metric * (d1 + d2 + d3) * metric;

                T v = -(point.vector().transpose() * right_jacobian.transpose() * derivative * right_jacobian * point.vector()).value();

                hessian.coeffRef(k2, k1) += v;

                if (k1 != k2)
                {
                    hessian.coeffRef(k1, k2) += v;
                }
            }

            Eigen::Vector<T, 5> g2 = -2.0 * right_jacobian.transpose() * metric * metric_gradients[k1] * metric * right_jacobian * point.vector();

            hessian.col(k1) += g2;
            hessian.row(k1) += g2.transpose();
        }

        hessian += 2.0 * right_jacobian.transpose() * metric * right_jacobian;

        return hessian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
    auto MultivectorGaussian<T, Multivector>::operator|(const MultivectorGaussian<T, Other> &other) const
    {
        using Gaussian = typename InnerProduct<Mean, Other<T>>::Type::Gaussian;

        auto operation = this->getMean() | other.getMean();

        auto left_jacobian = operation.getLeftJacobian();
        auto right_jacobian = operation.getRightJacobian();

        typename Gaussian::Mean mean = operation.evaluate();
        typename Gaussian::Covariance covariance = left_jacobian * this->getCovariance() * left_jacobian.transpose() +  //
                                                   right_jacobian * other.getCovariance() * right_jacobian.transpose();

        Gaussian gaussian;

        gaussian.setMean(mean);
        gaussian.setCovariance(covariance);

        return gaussian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
        requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
    auto MultivectorGaussian<T, Multivector>::operator|(const Other<T> &other) const
    {
        typename Other<T>::Gaussian gaussian;

        gaussian.setMean(other);
        gaussian.setCovariance(Other<T>::Gaussian::Covariance::Zero());

        return (*this) | gaussian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
    auto MultivectorGaussian<T, Multivector>::operator^(const MultivectorGaussian<T, Other> &other) const
    {
        using Gaussian = typename OuterProduct<Mean, Other<T>>::Type::Gaussian;

        auto operation = this->getMean() ^ other.getMean();

        auto left_jacobian = operation.getLeftJacobian();
        auto right_jacobian = operation.getRightJacobian();

        typename Gaussian::Mean mean = operation.evaluate();
        typename Gaussian::Covariance covariance = left_jacobian * this->getCovariance() * left_jacobian.transpose() +  //
                                                   right_jacobian * other.getCovariance() * right_jacobian.transpose();

        Gaussian gaussian;

        gaussian.setMean(mean);
        gaussian.setCovariance(covariance);

        return gaussian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
        requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
    auto MultivectorGaussian<T, Multivector>::operator^(const Other<T> &other) const
    {
        typename Other<T>::Gaussian gaussian;

        gaussian.setMean(other);
        gaussian.setCovariance(Other<T>::Gaussian::Covariance::Zero());

        return (*this) ^ gaussian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
    auto MultivectorGaussian<T, Multivector>::operator*(const MultivectorGaussian<T, Other> &other) const
    {
        using Gaussian = typename GeometricProduct<Mean, Other<T>>::Type::Gaussian;

        auto operation = this->getMean() * other.getMean();

        auto left_jacobian = operation.getLeftJacobian();
        auto right_jacobian = operation.getRightJacobian();

        typename Gaussian::Mean mean = operation.evaluate();
        typename Gaussian::Covariance covariance = left_jacobian * this->getCovariance() * left_jacobian.transpose() +  //
                                                   right_jacobian * other.getCovariance() * right_jacobian.transpose();

        Gaussian gaussian;

        gaussian.setMean(mean);
        gaussian.setCovariance(covariance);

        return gaussian;
    }

    template <class T, template <class S> class Multivector>
    template <template <class S> class Other>
        requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
    auto MultivectorGaussian<T, Multivector>::operator*(const Other<T> &other) const
    {
        typename Other<T>::Gaussian gaussian;

        gaussian.setMean(other);
        gaussian.setCovariance(Other<T>::Gaussian::Covariance::Zero());

        return (*this) * gaussian;
    }

    // template <class T, class Other, template <class> class Multivector>
    // // requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
    // auto operator*(const AbstractMultivector<Other> &other, const MultivectorGaussian<T, Multivector> &gaussian)
    // {
    //     typename Other::Gaussian second_gaussian;

    //     second_gaussian.setMean(other.derived());
    //     second_gaussian.setCovariance(Other::Gaussian::Covariance::Zero());

    //     return second_gaussian * gaussian;
    // }

    template <class T, template <class> class Multivector, template <class> class Other>
        requires(std::derived_from<Other<T>, AbstractMultivector<typename Other<T>::Type>>)
    auto operator*(const Other<T> &other, const MultivectorGaussian<T, Multivector> &gaussian)
    {
        typename Other<T>::Gaussian second_gaussian;

        second_gaussian.setMean(other.derived());
        second_gaussian.setCovariance(Other<T>::Gaussian::Covariance::Zero());

        return second_gaussian * gaussian;
    }

}  // namespace gafro::probabilistic