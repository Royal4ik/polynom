namespace Polinom_Library
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;

    public class Polynom
    {
        private readonly List<double> coefficients;

        public Polynom(IEnumerable<double> array)
        {
            this.coefficients = array.ToList();
            while ((this.Degree != 0) && (Math.Abs(this.coefficients[this.Degree]) < 0.000000001))
            {
                this.coefficients.RemoveAt(this.Degree);
            }
        }

        public int Degree
        {
            get
            {
                return this.coefficients.Count - 1;
            }
        }

        public double this[int index]
        {
            get
            {
                return this.coefficients[index];
            }

            set
            {
                if ((index == this.coefficients.Count - 1) && (Math.Abs(value) < 0.0000000000000001))
                {
                    this.coefficients.Remove(this.coefficients.Count);
                }

                this.coefficients[index] = value;
            }
        }
               
        public override string ToString()
        {
            var line = new StringBuilder();
            var sign = '+';
            for (var i = this.Degree; i > 0; i--)
            {
                var per = this.coefficients[i];
                if (per < 0)
                {
                    per = -per;
                    sign = '-';
                }

                if (i == this.Degree)
                {
                    sign = ' ';
                }

                line.Append(sign);
                line.Append(" ");
                line.Append(per);
                line.Append("*x^");
                line.Append(i);
                line.Append(" ");
                sign = '+';
            }

            if (this.coefficients[0] < 0)
            {
                sign = '-';
            }

            line.Append(sign);
            line.Append(" ");
            line.Append(Math.Abs(this.coefficients[0]));
            return line.ToString();
        }
        
        public IEnumerable<double> Calculate(double[] array)
        {
            var values = array.Select((d, i) => this.Calculate(array[i]));
            return values;
        }

        public double Calculate(double value)
        {
            var results = 0.0;

            for (var i = 0; i < this.Degree + 1; i++)
            {
                results += this.coefficients[i] * Math.Pow(value, i);
            }

            return results;
        }

        public static Polynom operator +(Polynom first, Polynom second)
        {
            var maxDegree = first.Degree >= second.Degree ? first.Degree : second.Degree;
            maxDegree++;
            var sum = new double[maxDegree];
            for (var i = 0; i < maxDegree; i++)
            {
                if (i > first.Degree)
                {
                    sum[i] = second.coefficients[i];
                }
                else if (i > second.Degree)
                {
                    sum[i] = first.coefficients[i];
                }
                else
                {
                    sum[i] = first.coefficients[i] + second.coefficients[i];
                }
            }

            var summ = new Polynom(sum);
            return summ;
        }

        public static Polynom operator -(Polynom first, Polynom second)
        {
            if (first == null)
            {
                throw new ArgumentNullException("first");
            }

            var maxDegree = first.Degree >= second.Degree ? first.Degree : second.Degree;

            /*else
            {
                maxDegree = first.Degree;
                while (Math.Abs(first.coefficients[maxDegree] - second.coefficients[maxDegree]) < 0.00001)
                {
                    maxDegree--;
                    if (maxDegree >= 0)
                    {
                        continue;
                    }

                    maxDegree = 0;
                    break;
                }
            }*/

            var diff = new double[maxDegree + 1];
            for (var i = 0; i < maxDegree + 1; i++)
            {
                if (i > first.Degree)
                {
                    diff[i] = -second.coefficients[i];
                }
                else if (i > second.Degree)
                {
                    diff[i] = first.coefficients[i];
                }
                else
                {
                    diff[i] = first.coefficients[i] - second.coefficients[i];
                }
            }

            var difference = new Polynom(diff);
            return difference;
        }

        public static Polynom operator *(Polynom first, double multiplier)
        {
            var multiplicand = first.coefficients.Select(i => multiplier * i).ToList();
            return new Polynom(multiplicand);
        }


        public static Polynom operator *(Polynom first, Polynom second)
        {
            var maxDegree = first.Degree + second.Degree;
            var compostion = new double[maxDegree + 1];

            for (var i = 0; i < first.Degree + 1; i++)
            {
                for (var j = 0; j < second.Degree + 1; j++)
                {
                    compostion[j + i] += first.coefficients[i] * second.coefficients[j];
                }
            }

            var result1 = new Polynom(compostion);
            return result1;
        }
    }
}