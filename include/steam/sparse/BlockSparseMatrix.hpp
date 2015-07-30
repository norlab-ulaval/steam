//////////////////////////////////////////////////////////////////////////////////////////////
/// \file BlockSparseMatrix.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_BLOCK_SPARSE_MATRIX_HPP
#define STEAM_BLOCK_SPARSE_MATRIX_HPP

#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace steam {

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Class to store a block-sparse matrix
/////////////////////////////////////////////////////////////////////////////////////////////
class BlockSparseMatrix
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor, matrix size must still be set before using
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockSparseMatrix(bool square = false, bool symmetric = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Square matrix constructor, symmetry is still optional
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockSparseMatrix(const std::vector<unsigned int>& blkSqSizes, bool symmetric = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Rectangular matrix constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockSparseMatrix(const std::vector<unsigned int>& blkRowSizes, const std::vector<unsigned int>& blkColSizes);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Resize and clear matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void reset(const std::vector<unsigned int>& blkRowSizes, const std::vector<unsigned int>& blkColSizes);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Resize and clear a square matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void reset(const std::vector<unsigned int>& blkSizes);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Clear sparse entries, maintain size
  //////////////////////////////////////////////////////////////////////////////////////////////
  void clear();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get number of block rows
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int getNumBlkRows() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get number of block columns
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int getNumBlkCols() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Adds the matrix to the block entry at index (r,c), block dim must match
  //////////////////////////////////////////////////////////////////////////////////////////////
  void add(unsigned int r, unsigned int c, const Eigen::MatrixXd& m);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns a const reference to the block entry at index (r,c)
  //////////////////////////////////////////////////////////////////////////////////////////////
  const Eigen::MatrixXd& read(unsigned int r, unsigned int c);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Convert to Eigen sparse matrix format
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::SparseMatrix<double> toEigen(bool getSubBlockSparsity = false) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Gets the number of non-zero entries per column (helper for prealloc. Eigen Sparse)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXi getNnzPerCol() const;

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
  /// \brief Whether matrix is square
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool square_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether matrix is symmetric
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool symmetric_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Total scalar size in row dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int scalarRowDim_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Total scalar size in column dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int scalarColDim_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Total 'block' size in row dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<unsigned int> blkRowSizes_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Total 'block' size in column dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<unsigned int> blkColSizes_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cumulative sum of scalar matrix size (across blocks) in the row dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<unsigned int> cumBlkRowSizes_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cumulative sum of scalar matrix size (across blocks) in the column dimension
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<unsigned int> cumBlkColSizes_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Vector of columns (the data container)
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<BlockSparseColumn> cols_;

};

} // steam

#endif // STEAM_BLOCK_SPARSE_MATRIX_HPP
