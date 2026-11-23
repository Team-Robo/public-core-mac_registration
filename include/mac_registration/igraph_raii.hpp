// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// Small owners for igraph C objects: each frees its object when it goes out of scope, so no
// early return in the solver can leak (upstream leaks its weight vectors and, on its
// "no cliques" path, the graph and matrix).
#pragma once

#include <igraph/igraph.h>

#include <stdexcept>
#include <string>

namespace mac
{
namespace ig
{

inline void check(igraph_error_t rc, const char * what)
{
  if (rc != IGRAPH_SUCCESS) {
    throw std::runtime_error(std::string("igraph: ") + what + " failed: " + igraph_strerror(rc));
  }
}

struct Matrix
{
  igraph_matrix_t m;
  Matrix(igraph_integer_t rows, igraph_integer_t cols)
  {
    check(igraph_matrix_init(&m, rows, cols), "igraph_matrix_init");
  }
  ~Matrix() {igraph_matrix_destroy(&m);}
  Matrix(const Matrix &) = delete;
  Matrix & operator=(const Matrix &) = delete;
};

struct Vector
{
  igraph_vector_t v;
  explicit Vector(igraph_integer_t n = 0) {check(igraph_vector_init(&v, n), "igraph_vector_init");}
  ~Vector() {igraph_vector_destroy(&v);}
  Vector(const Vector &) = delete;
  Vector & operator=(const Vector &) = delete;
};

struct IntList
{
  igraph_vector_int_list_t l;
  IntList() {check(igraph_vector_int_list_init(&l, 0), "igraph_vector_int_list_init");}
  ~IntList() {igraph_vector_int_list_destroy(&l);}
  IntList(const IntList &) = delete;
  IntList & operator=(const IntList &) = delete;
};

struct Graph
{
  igraph_t g;
  bool valid{false};
  Graph() = default;
  ~Graph()
  {
    if (valid) {igraph_destroy(&g);}
  }
  Graph(const Graph &) = delete;
  Graph & operator=(const Graph &) = delete;
};

}  // namespace ig
}  // namespace mac
