#include <gafro/gafro.hpp>

using namespace gafro;

struct DQMetric
{
    constexpr static int dim = 3;

    template <int i, int j>
    constexpr static double get()
    {
        if constexpr (i == j)
        {
            if (i < 2)
            {
                return -1.0;
            }
            else
            {
                return 0.0;
            }
        }

        return 0.0;
    }
};

using DQ = Algebra<DQMetric>;
template <int... i>
using M = DQ::Multivector<double, i...>;

constexpr static std::array<int, 8> dq_blades = { 0b000, 0b001, 0b010, 0b100, 0b011, 0b101, 0b110, 0b111 };
constexpr static std::array<std::string, 8> dq_blade_names = { "", "e1", "e2", "e12", "e3", "e13", "e23", "e123" };
// constexpr static std::array<std::string, 8> dq_blade_names = { "", "e", "i", "ei", "j", "ej", "k", "ek" };

template <int... index>
std::ostream &operator<<(std::ostream &ostream, const M<index...> &mv)
{
    if (sizeof...(index) == 0)
    {
        ostream << 0;

        return ostream;
    }

    bool first = true;

    for (unsigned int k = 0; k < mv.vector().rows(); ++k)
    {
        if (abs(mv.vector().coeff(k, 0)) < 1e-10)
        {
            continue;
        }

        if (!first)
        {
            ostream << (mv.vector().coeff(k, 0) >= 0 ? " + " : " - ");

            ostream << abs(mv.vector().coeff(k, 0));
        }
        else
        {
            ostream << mv.vector().coeff(k, 0);

            first = false;
        }

        if (M<index...>::blades()[k] > 0)
        {
            ostream << "*" << dq_blade_names[M<index...>::blades()[k]];
        }
    }

    if (first)
    {
        ostream << 0;
    }

    return ostream;
}

template <int b1, int... b2s>
struct Compare;

template <int b1, int b2, int... b2s>
struct Compare<b1, b2, b2s...>
{
    static void show()
    {
        std::cout << (M<dq_blades[b1]>(1.0) * M<dq_blades[b2]>(1.0)).evaluate() << "\t";
    }

    static void display()
    {
        show();

        (Compare<b1, b2s>::show(), ...);

        std::cout << std::endl;
    }
};

template <std::size_t... i, std::size_t... j>
void table(std::index_sequence<i...>, std::index_sequence<j...>)
{
    (Compare<i, j...>::display(), ...);
}

int main(int /*argc*/, char ** /*argv*/)
{
    table(std::make_index_sequence<8>(), std::make_index_sequence<8>());

    std::cout << DQMetric::get<0, 0>() << " " << DQMetric::get<0, 1>() << " " << DQMetric::get<0, 2>() << std::endl;
    std::cout << DQMetric::get<1, 0>() << " " << DQMetric::get<1, 1>() << " " << DQMetric::get<1, 2>() << std::endl;
    std::cout << DQMetric::get<2, 0>() << " " << DQMetric::get<2, 1>() << " " << DQMetric::get<2, 2>() << std::endl;

    return 0;
}
