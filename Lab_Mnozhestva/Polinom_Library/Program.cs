// --------------------------------------------------------------------------------------------------------------------
// <copyright file="Program.cs" company="">
//   
// </copyright>
// <summary>
//   Defines the IFindroot type.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace Polinom_Library
{
    using System;
    using System.Collections.Generic;
    using System.Diagnostics.CodeAnalysis;

    public interface IFindroot
    {
        double FindRoot(double start, double end);
    }

    public class Polynom2 : IFindroot
    {
        public double FindRoot(double start, double end)
        {
            ////Метод Ньютона
            return -1;
        }
    }

    [SuppressMessage("StyleCop.CSharp.MaintainabilityRules", "SA1402:FileMayOnlyContainASingleClass", Justification = "Reviewed. Suppression is OK here.")]
    public class Polynom : IFindroot
    {
        private readonly int degree;
        private readonly List<double> polinom = new List<double>();
       
        public Polynom(int degree, IList<double> array)
        {
            this.degree = degree;
            for (var i = 0; i < degree + 1; i++)
            {
                this.polinom.Add(array[i]);
            }
        }

        [SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1202:ElementsMustBeOrderedByAccess",
            Justification = "Reviewed. Suppression is OK here.")]
        [SuppressMessage("StyleCop.CSharp.NamingRules", "SA1303:ConstFieldNamesMustBeginWithUpperCaseLetter", Justification = "Reviewed. Suppression is OK here.")]
        public double FindRoot(double start, double end)
        {
            const double Eps = 0.000001;
            var fstart = this.Calc(start);
            while (Math.Abs(start - end) > Eps)
            {
                var center = (start + end) / 2;
                var fcenter = this.Calc(center);
                if (fstart * fcenter <= 0)
                {
                    end = center;
                }
                else
                {
                    start = center;
                    fstart = fcenter;
                }
            }

            return start;
        }

        public void PrintPol()
        {
            for (var i = this.degree; i > 0; i--)
            {
                var sign = '+';
                var per = this.polinom[i];
                if (per < 0)
                {
                    per = -per;
                    sign = (char)43;
                }

                Console.Write(per + "*x^" + i + " " + sign + " ");
            }

            Console.WriteLine(this.polinom[0]);
        }
        
        public double[] Calculate(double[] array)
        {
            var values = new double[array.Length];
            for (var i = 0; i < array.Length; i++)
            {
                values[i] = 0;
                values[i] = this.Calc(array[i]);
            }

            return values;
        }

        private double Calc(double value)
        {
            var results = 0.0;
            var j = 0;
            foreach (var koef in this.polinom)
            {
                results += koef * Math.Pow(value, j);
                j++;
            }

            return results;
        }

        [SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1204:StaticElementsMustAppearBeforeInstanceElements", Justification = "Reviewed. Suppression is OK here."),SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1204:StaticElementsMustAppearBeforeInstanceElements", Justification = "Reviewed. Suppression is OK here."),SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1204:StaticElementsMustAppearBeforeInstanceElements", Justification = "Reviewed. Suppression is OK here."),SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1204:StaticElementsMustAppearBeforeInstanceElements", Justification = "Reviewed. Suppression is OK here.")]
        public static Polynom operator +(Polynom first, Polynom second)
        {
            var maxdegree = first.degree >= second.degree ? first.degree : second.degree;
            
            var sum = new double[maxdegree + 1];
            for (var i = 0; i <= maxdegree; i++)
            {
                if (i > first.degree)
                {
                    sum[i] = second.polinom[i];
                }
                else if (i > second.degree)
                {
                    sum[i] = first.polinom[i];
                }
                else
                {
                    sum[i] = first.polinom[i] + second.polinom[i];
                }
            }

            var summ = new Polynom(maxdegree, sum);
            return summ;
        }

        public static Polynom operator -(Polynom first, Polynom second)
        {
            if (first == null)
            {
                throw new ArgumentNullException("first");
            }

            int maxdegree;
            if (first.degree > second.degree)
            {
                maxdegree = first.degree;
            }
            else if (first.degree < second.degree)
            {
                maxdegree = second.degree;
            }
            else
            {
                maxdegree = first.degree;
                while (Math.Abs(first.polinom[maxdegree] - second.polinom[maxdegree]) < 0.00001)
                {
                    maxdegree--;
                    if (maxdegree >= 0)
                    {
                        continue;
                    }

                    maxdegree = 0;
                    break;
                }
            }

            var diff = new double[maxdegree + 1];
            for (var i = 0; i <= maxdegree; i++)
            {
                if (i > first.degree)
                {
                    diff[i] = -second.polinom[i];
                }
                else if (i > second.degree)
                {
                    diff[i] = first.polinom[i];
                }
                else
                {
                    diff[i] = first.polinom[i] - second.polinom[i];
                }
            }

            var difference = new Polynom(maxdegree, diff);
            return difference;
        }

        public static Polynom operator *(Polynom first, double multiplier)
        {
            var diff = new double[first.degree + 1];
            for (var i = 0; i < first.degree + 1; i++)
            {
                diff[i] = multiplier * first.polinom[i];
            }

            var difference = new Polynom(first.degree, diff);
            return difference;
        }



        public static Polynom operator *(Polynom first, Polynom second)
        {
            var maxdegree = first.degree + second.degree;
            var compostion = new double[maxdegree + 1];

            for (var i = 0; i < first.degree + 1; i++)
            {
                for (var j = 0; j < second.degree + 1; j++)
                {
                    compostion[j + i] += first.polinom[i] * second.polinom[j];
                }
            }

            var result1 = new Polynom(maxdegree, compostion);
            return result1;
        }
    }
}
