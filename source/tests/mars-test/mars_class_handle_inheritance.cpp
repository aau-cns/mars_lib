#include <gmock/gmock.h>
#include <Eigen/Dense>

#include <mars/sensors/sensor_interface.h>
#include <vector>

class class_handle_inheritance : public testing::Test
{
public:
};

class A
{
public:
  virtual int function_1(const int& param_1) = 0;
  virtual int function_2(const int& param_2) = 0;
};

class B : public A
{
public:
  int member_1;
  int member_2;

  B() : member_1(10), member_2(12)
  {
  }

  int function_1(const int& param_1)
  {
    return param_1 * 3;
  }

  int function_2(const int& param_2)
  {
    return param_2 * 4;
  }
};

class C : public A
{
public:
  int member_1;
  int member_2;

  C() : member_1(10), member_2(12)
  {
  }

  int function_1(const int& param)
  {
    return some_private_function(param);
  }

  int function_2(const int& param)
  {
    return another_private_function(param);
  }

private:
  int some_private_function(const int& param)
  {
    return param * 5;
  }

  int another_private_function(const int& param)
  {
    return param * 10;
  }
};

template <class T>
class D : public A
{
public:
  int member_1;
  int member_2;

  D() : member_1(10), member_2(12)
  {
  }

  int function_1(const int& param)
  {
    return some_private_function(param);
  }

  int function_2(const int& param)
  {
    return another_private_function(param);
  }

private:
  int some_private_function(const T& param)
  {
    return param * 5;
  }

  int another_private_function(const T& param)
  {
    return param * 10;
  }
};

TEST_F(class_handle_inheritance, CTOR)
{
  B b_instance;
  C c_instance;
  D<double> d_instance;
  std::vector<A*> test_vector;

  test_vector.push_back(&b_instance);
  test_vector.push_back(&c_instance);
  test_vector.push_back(&d_instance);

  ASSERT_EQ(test_vector[0]->function_1(5), 15);
  ASSERT_EQ(test_vector[0]->function_2(10), 40);
  ASSERT_EQ(test_vector[1]->function_1(5), 25);
  ASSERT_EQ(test_vector[1]->function_2(5), 50);
  ASSERT_EQ(test_vector[2]->function_1(5), 25);
  ASSERT_EQ(test_vector[2]->function_2(5), 50);
}
