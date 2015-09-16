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
                if ((index == this.coefficients.Count - 1) && (value == 0))
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
            for (var i = this.Degree - 1; i > 0; i--)
            {
                var per = this.coefficients[i];
                if (per < 0)
                {
                    per = -per;
                    sign = '-';
                }

                if (i == this.Degree - 1)
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
        
        public double[] Calculate(double[] array)
        {
            var values = new double[array.Length];
            
            for (var i = 0; i < array.Length; i++)
            {
                values[i] = this.Calculate(array[i]);
            }

            return values;
        }

        public double Calculate(double value)
        {
            var results = 0.0;

            for (var i = 0; i < this.Degree; i++)
            {
                results += this.coefficients[i] * Math.Pow(value, i);
            }

            return results;
        }
      
        public static Polynom operator +(Polynom first, Polynom second)
        {

            var maxDegree = first.Degree >= second.Degree ? first.Degree : second.Degree;
            
            var sum = new double[maxDegree];
            for (var i = 0; i < maxDegree; i++)
            {
                if (i >= first.Degree)
                {
                    sum[i] = second.coefficients[i];
                }
                else if (i >= second.Degree)
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

            int maxDegree;
            if (first.Degree > second.Degree)
            {
                maxDegree = first.Degree;
            }
            else if (first.Degree < second.Degree)
            {
                maxDegree = second.Degree;
            }
            else
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
            }

            var diff = new double[maxDegree];
            for (var i = 0; i < maxDegree; i++)
            {
                if (i >= first.Degree)
                {
                    diff[i] = -second.coefficients[i];
                }
                else if (i >= second.Degree)
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
            // TODO: почему difference?
            // исправил
            var multiplicand = new double[first.Degree];

            // TODO: ¬место цикла for  логичнее использовать foreach
            // дл€ multiplicand отдельно считать индеск считать?
            for (var i = 0; i < first.Degree; i++)
            {
                multiplicand[i] = multiplier * first.coefficients[i];
            }

            var multiply = new Polynom(multiplicand);
            return multiply;
        }


        public static Polynom operator *(Polynom first, Polynom second)
        {
            var maxDegree = first.Degree + second.Degree;
            var compostion = new double[maxDegree - 1];

            for (var i = 0; i < first.Degree; i++)
            {
                for (var j = 0; j < second.Degree; j++)
                {
                    compostion[j + i] += first.coefficients[i] * second.coefficients[j];
                }
            }

            var result1 = new Polynom(compostion);
            return result1;
        }
    }
}