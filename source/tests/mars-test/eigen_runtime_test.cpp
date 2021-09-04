#include <gmock/gmock.h>
#include <Eigen/Dense>
#include <chrono>


class eigen_runtime_test: public testing::Test
{
  public:
};

template<int dim=3>
void test_rt(int const num_iterations)
{
  Eigen::MatrixXd M1 =  Eigen::MatrixXd::Random(dim,dim);
  Eigen::Matrix<double, dim, dim> M_stat = Eigen::Matrix<double, dim, dim>::Random();

  typedef std::chrono::high_resolution_clock clk_t;
  typedef clk_t::time_point tp_t;
  {
    tp_t t1 = clk_t::now();

    for(int i=0; i < num_iterations; i++)
    {
      M1 = M1*M1;
    }
    tp_t t2 = clk_t::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "["<<dim << "x"<< dim << "] Dynamic * Dynamic avg: " << duration/(1.0*num_iterations) << " us" << std::endl;
  }


  {
    tp_t t1 = clk_t::now();

    for(int i=0; i < num_iterations; i++)
    {
      M_stat = M_stat*M_stat;
    }
    tp_t t2 = clk_t::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "["<<dim << "x"<< dim << "] Static * Static avg: " << duration/(1.0*num_iterations) << " us" << std::endl;
  }

  {
    tp_t t1 = clk_t::now();

    for(int i=0; i < num_iterations; i++)
    {
      M_stat = M1*M_stat;
    }
    tp_t t2 = clk_t::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "["<<dim << "x"<< dim << "] Dynamic * Static avg: " << duration/(1.0*num_iterations) << " us" << std::endl;
  }
}

TEST_F(eigen_runtime_test, time_measurement_3)
{
  test_rt<3>(100000);
}

TEST_F(eigen_runtime_test, time_measurement_20)
{
  test_rt<20>(10000);
}

TEST_F(eigen_runtime_test, time_measurement_100)
{
  test_rt<100>(100);
}
