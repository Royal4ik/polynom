namespace Polinom_Library
{
    using System;
    using System.Collections.Generic;
    using System.Diagnostics.CodeAnalysis;
    using System.Linq.Expressions;

    // TODO: Каждый отдельый класс и интерфейс выносится в одноименный файл. 
    // TODO: Один файл - одна сущность. В проекте с библиотеками класс Program равно как и файл Program обычно отсутствует
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

    public class Polynom : IFindroot
    {
        private readonly List<double> polinom = new List<double>();

        // TODO: Зачем передавать степень, если она на прямую зависит от набора коэффициентов. Лучше сделать её свойством и вычислять диамически.
        public Polynom(IEnumerable<double> array)
        {
            foreach (var t in array)
            {
                this.polinom.Add(t);
            }
        }

        private int Degree
        {
            get { return this.polinom.Count; }
        }

        public double FindRoot(double start, double end)
        {
            const double Eps = 0.000001;
            var fstart = this.Calculate(start);
            while (Math.Abs(start - end) > Eps)
            {
                var center = (start + end) / 2;
                var fcenter = this.Calculate(center);
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

        // TODO: Так как мы обращаемся к этому методу в контексте многочлена, то есть (polynom.PrintPol()),
        // TODO: то логичнее назвать его просто Print, или даже как вариант, перегрузить метод ToString()
        // исправил
        public override string ToString()
        {
            var line = " ";
            var sign = '+';
            for (var i = this.Degree - 1; i > 0; i--)
            {
                var per = this.polinom[i];
                if (per < 0)
                {
                    per = -per;

                    // TODO: это зачем вообще? У тебя знак всегда равен плюс. Убрать.
                    // так то нет, знак может быть минус
                    sign = (char)45;
                }

                // TODO: Убрать использование методов консоли. Полином не должен от нее зависеть. Переписать, чтобы этот метод возвращал строку
                // исправил
                if (i == this.Degree - 1)
                {
                    sign = ' ';
                }
                line += sign + " " + per + "*x^" + i + " ";
                sign = '+';
            }

            if (this.polinom[0] < 0)
            {
                sign = (char)45;
            }

            line += sign + " " + Math.Abs(this.polinom[0]);
            return line;
        }
        
        public double[] Calculate(double[] array)
        {
            var values = new double[array.Length];
            
            for (var i = 0; i < array.Length; i++)
            {
                 // TODO: зачем?
                // исправил
                values[i] = this.Calculate(array[i]);
            }

            return values;
        }

        // TODO: Вместо использования корявых имен (чего вообще делать нельзя), логичнее использовать перегрузку метода Calculate для одиночного значения параметра.
        // исправил
        private double Calculate(double value)
        {
            var results = 0.0;

            // TODO: Если требуется индекс элемента, лучше использовать for, а не foreach (это единственный случай, когда метод for лучше)
            // исправил
            for (var i = 0; i < this.Degree; i++)
            {
                results += this.polinom[i] * Math.Pow(value, i);
            }

            return results;
        }
      
        public static Polynom operator +(Polynom first, Polynom second)
        {
            // TODO: Мы придерживаемся "верблюжьей" конвенции именования. maxDegree. Новое слово - большая буква
            // исправил
            var maxDegree = first.Degree >= second.Degree ? first.Degree : second.Degree;
            
            var sum = new double[maxDegree];
            for (var i = 0; i < maxDegree; i++)
            {
                if (i >= first.Degree)
                {
                    sum[i] = second.polinom[i];
                }
                else if (i >= second.Degree)
                {
                    sum[i] = first.polinom[i];
                }
                else
                {
                    sum[i] = first.polinom[i] + second.polinom[i];
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

            var maxDegree = 0;
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
                while (Math.Abs(first.polinom[maxDegree] - second.polinom[maxDegree]) < 0.00001)
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
                    diff[i] = -second.polinom[i];
                }
                else if (i >= second.Degree)
                {
                    diff[i] = first.polinom[i];
                }
                else
                {
                    diff[i] = first.polinom[i] - second.polinom[i];
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

            // TODO: Вместо цикла for  логичнее использовать foreach
            // для multiplicand отдельно считать индеск считать?
            for (var i = 0; i < first.Degree; i++)
            {
                multiplicand[i] = multiplier * first.polinom[i];
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
                    compostion[j + i] += first.polinom[i] * second.polinom[j];
                }
            }

            var result1 = new Polynom(compostion);
            return result1;
        }
    }
}
