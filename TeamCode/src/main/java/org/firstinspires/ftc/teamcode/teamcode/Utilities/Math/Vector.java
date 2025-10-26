// Primary Author:
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

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.NonConst;

import java.util.Arrays;
import java.util.Random;

/**
 * A {@link Vector} represents a single-dimensional vector of doubles. It is <em>not</em> a matrix,
 * but can easily be converted into either a {@link RowMatrix} or a {@link ColumnMatrix} should
 * that be desired. That said, vectors can be multiplied by matrices to their left (or right); this
 * is commonly used to transform a set of coordinates (in the vector) by a transformation matrix.
 *
 * @see Matrix
 * @see RowMatrix
 * @see ColumnMatrix
 */
public class Vector
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected double[] data;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Creates a new vector of the indicated length. The vector will contain zeros.
     * @param length the length of the new vector to return
     * @return the newly created vector
     */
    public static Vector length(int length)
    {
        return new Vector(new double[length]);
    }

    public Vector(double[] data)
    {
        this.data = data;
    }

    public Vector(double x)
    {
        this.data = new double[1];
        this.data[0] = x;
    }

    public Vector(double x, double y)
    {
        this.data = new double[2];
        this.data[0] = x;
        this.data[1] = y;
    }

    public Vector(double x, double y, double z)
    {
        this.data = new double[3];
        this.data[0] = x;
        this.data[1] = y;
        this.data[2] = z;
    }

    public Vector(double x, double y, double z, double w)
    {
        this.data = new double[4];
        this.data[0] = x;
        this.data[1] = y;
        this.data[2] = z;
        this.data[3] = w;
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    @Const public double[] getData()
    {
        return this.data;
    }

    @Const public int length()
    {
        return this.data.length;
    }

    @Const public double get(int index)
    {
        return this.data[index];
    }

    @NonConst public void put(int index, double value)
    {
        this.data[index] = value;
    }

    public void add(int index, double value) {
        this.data[index] += value;
    }

    @Override public String toString()
    {
        StringBuilder result = new StringBuilder();
        result.append("{");
        for (int i = 0; i < this.length(); i++)
        {
            if (i > 0) result.append(" ");
            result.append(String.format("%.2f", this.data[i]));
        }
        result.append("}");
        return result.toString();
    }

    //----------------------------------------------------------------------------------------------
    // Transformation matrix operations
    //----------------------------------------------------------------------------------------------

    /**
     * Consider this vector as a 3D coordinate or 3D homogeneous coordinate, and, if the
     * latter, return its normalized form. In either case, the result is of length three, and
     * contains coordinate values for x, y, and z at indices 0, 1, and 2 respectively.
     * @return the normalized form of this coordinate vector
     *
     * @see <a href="https://en.wikipedia.org/wiki/Homogeneous_coordinates">Homogeneous coordinates</a>
     */
    @Const public Vector normalized3D()
    {
        if (this.length()==3)
        {
            return this;
        }
        else if (this.length()==4)
        {
            return new Vector(
                    this.data[0]/this.data[3],
                    this.data[1]/this.data[3],
                    this.data[2]/this.data[3]);
        }
        else
            throw dimensionsError();
    }

    // Sets vector magnitude to 1
    @Const public Vector normalized() {
        return this.multiplied(1/this.magnitude());
    }

    //----------------------------------------------------------------------------------------------
    // Matrix Operations
    //----------------------------------------------------------------------------------------------

    @Const public double magnitude()
    {
        return (double)Math.sqrt(this.dotProduct(this));
    }

    /**
     * Returns the dot product of this vector and another.
     * @param him the other vector with whom the dot product is to be formed
     * @return the dot product of this vector and another.
     *
     * @see <a href="https://en.wikipedia.org/wiki/Dot_product">Dot product</a>
     */
    @Const public double dotProduct(Vector him)
    {
        if (this.length() == him.length())
        {
            double sum = 0;
            for (int i = 0; i < this.length(); i++)
            {
                sum += this.get(i) * him.get(i);
            }
            return sum;
        }
        else
            throw dimensionsError();
    }

    /**
     * Multiplies this vector, taken as a row vector, against the indicated matrix.
     */
    @Const public Matrix multiplied(Matrix him)
    {
        return new RowMatrix(this).multiplied(him);
    }

    /**
     * Adds this vector, taken as a row vector against, to the indicated matrix.
     */
    @Const public Matrix added(Matrix addend)
    {
        return new RowMatrix(this).added(addend);
    }

    @Const public Vector added(Vector addend)
    {
        if (this.length() == addend.length())
        {
            Vector result = Vector.length(this.length());
            for (int i = 0; i < this.length(); i++)
            {
                result.put(i, this.get(i) + addend.get(i));
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    @NonConst public void add(Vector addend)
    {
        if (this.length() == addend.length())
        {
            for (int i = 0; i < this.length(); i++)
            {
                this.put(i, this.get(i) + addend.get(i));
            }
        }
        else
            throw dimensionsError();
    }

    /**
     * Subtracts the indicated matrix from this vector, taken as a row vector.
     */
    @Const public Matrix subtracted(Matrix subtrahend)
    {
        return new RowMatrix(this).subtracted(subtrahend);
    }

    @Const public Vector subtracted(Vector subtrahend)
    {
        if (this.length() == subtrahend.length())
        {
            Vector result = Vector.length(this.length());
            for (int i = 0; i < this.length(); i++)
            {
                result.put(i, this.get(i) - subtrahend.get(i));
            }
            return result;
        }
        else
            throw dimensionsError();
    }

    @NonConst public void subtract(Vector subtrahend)
    {
        if (this.length() == subtrahend.length())
        {
            for (int i = 0; i < this.length(); i++)
            {
                this.put(i, this.get(i) - subtrahend.get(i));
            }
        }
        else
            throw dimensionsError();
    }

    /**
     * Returns a new vector containing the elements of this vector scaled by the indicated factor.
     */
    @Const public Vector multiplied(double scale)
    {
        Vector result = Vector.length(this.length());
        for (int i = 0; i < this.length(); i++)
        {
            result.put(i, this.get(i) * scale);
        }
        return result;
    }

    @NonConst public void multiply(double scale)
    {
        for (int i = 0; i < this.length(); i++)
        {
            this.put(i, this.get(i) * scale);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    protected RuntimeException dimensionsError()
    {
        return dimensionsError(this.length());
    }

    @SuppressLint("DefaultLocale") protected static RuntimeException dimensionsError(int length)
    {
        return new IllegalArgumentException(String.format("vector dimensions are incorrect: length=%d", length));
    }

    public static Vector random(int size, double min, double max) {
        Random rand = new Random();
        double[] values = new double[size];

        for (int i = 0; i < size; i++) {
            values[i] = rand.nextDouble() * (max-min) + min;
        }

        return new Vector(values);
    }

    public static Vector withValue(double value, int size) {
        double[] values = new double[size];

        Arrays.fill(values, value);

        return new Vector(values);
    }

}
