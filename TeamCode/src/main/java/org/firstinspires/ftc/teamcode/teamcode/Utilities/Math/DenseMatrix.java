// Primary Author: FTC SDK / Robert Atkinson
// Minor adjustments by Kieran Mattingly
/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Math;

/**
 * A {@link DenseMatrix} is a matrix of doubles whose storage is a contiguous double[] array. It may
 * logically be ranged arranged either in row or column major order.
 *
 * @see Matrix
 * @see RowMajorMatrix
 */
public abstract class DenseMatrix extends Matrix
{
    protected DenseMatrix(int nRows, int nCols)
    {
        super(nRows, nCols);
    }

    @Override public double get(int row, int col)
    {
        return getData()[indexFromRowCol(row, col)];
    }

    @Override public void put(int row, int col, double value)
    {
        getData()[indexFromRowCol(row, col)] = value;
    }

    /**
     * Returns the contiguous array of doubles which is the storage for this matrix
     * @return the contiguous array of doubles which is the storage for this matrix
     */
    public abstract double[] getData();

    /**
     * Given a row and column index into the matrix, returns the corresponding index
     * into the underlying double[] array.
     * @param row   the row whose index is desired
     * @param col   the column whose index is desired
     * @return the index of (row,col) in the data returned by {@link #getData()}
     */
    protected abstract int indexFromRowCol(int row, int col);
}
