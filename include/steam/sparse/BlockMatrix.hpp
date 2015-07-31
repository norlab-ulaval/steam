//////////////////////////////////////////////////////////////////////////////////////////////
/// \file BlockMatrix.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_BLOCK_MATRIX_HPP
#define STEAM_BLOCK_MATRIX_HPP

#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <steam/sparse/BlockMatrixBase.hpp>

namespace steam {

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Class to store a block matrix
/////////////////////////////////////////////////////////////////////////////////////////////
class BlockMatrix : public BlockMatrixBase
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor, matrix size must still be set before using
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockMatrix(bool square = false, bool symmetric = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Rectangular matrix constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockMatrix(const std::vector<unsigned int>& blkRowSizes,
              const std::vector<unsigned int>& blkColSizes);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Square matrix constructor, symmetry is still optional
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockMatrix(const std::vector<unsigned int>& blkSqSizes, bool symmetric = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Set entries to zero
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void zero();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Adds the matrix to the block entry at index (r,c), block dim must match
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void add(unsigned int r, unsigned int c, const Eigen::MatrixXd& m);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns a reference to the value at (r,c), if it exists
  ///        *Note this throws an exception if matrix is symmetric and you request a lower
  ///         triangular entry. For read operations, use copyAt(r,c).
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::MatrixXd& at(unsigned int r, unsigned int c);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns a copy of the entry at index (r,c)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::MatrixXd copyAt(unsigned int r, unsigned int c) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Private row entry
  //////////////////////////////////////////////////////////////////////////////////////////////
  struct BlockRowEntry
  {
    Eigen::MatrixXd data;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Private column structure (holds a list of row entries)
  //////////////////////////////////////////////////////////////////////////////////////////////
  struct BlockSparseColumn
  {
    std::map<unsigned int, BlockRowEntry> rows;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Vector of columns (the data container)
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<BlockSparseColumn> cols_;

};

} // steam

#endif // STEAM_BLOCK_MATRIX_HPP
