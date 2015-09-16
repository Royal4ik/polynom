namespace Polinom_Library
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;

    public class Polynom
    {
        private readonly List<double> coefficient = new List<double>();

        // TODO: Зачем передавать степень, если она на прямую зависит от набора коэффициентов. Лучше сделать её свойством и вычислять диамически.
        public Polynom(IEnumerable<double> array)
        {
            this.coefficient = array.ToList();
        }

        public int Degree
        {
            get { return this.coefficient.Count; }
        }

        public double this[int index]
        {
            get
            {
                return this.coefficient[index];
            }

            set
            {
                this.coefficient[index] = value;
            }
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
            var line = new StringBuilder();
            var sign = '+';
            for (var i = this.Degree - 1; i > 0; i--)
            {
                var per = this.coefficient[i];
                if (per < 0)
                {
                    per = -per;

                    // TODO: это зачем вообще? У тебя знак всегда равен плюс. Убрать.
                    // так то нет, знак может быть минус
                    sign = '-';
                }

                // TODO: Убрать использование методов консоли. Полином не должен от нее зависеть. Переписать, чтобы этот метод возвращал строку
                // исправил
                if (i == this.Degree - 1)
                {
                    sign = ' ';
                }

                line.Append(sign + " " + per + "*x^" + i + " ");
                sign = '+';
            }

            if (this.coefficient[0] < 0)
            {
                sign = (char)45;
            }

            line.Append(sign + " " + Math.Abs(this.coefficient[0]));
            return line.ToString();
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
        public double Calculate(double value)
        {
            var results = 0.0;

            // TODO: Если требуется индекс элемента, лучше использовать for, а не foreach (это единственный случай, когда метод for лучше)
            // исправил
            for (var i = 0; i < this.Degree; i++)
            {
                results += this.coefficient[i] * Math.Pow(value, i);
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
                    sum[i] = second.coefficient[i];
                }
                else if (i >= second.Degree)
                {
                    sum[i] = first.coefficient[i];
                }
                else
                {
                    sum[i] = first.coefficient[i] + second.coefficient[i];
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
                while (Math.Abs(first.coefficient[maxDegree] - second.coefficient[maxDegree]) < 0.00001)
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
                    diff[i] = -second.coefficient[i];
                }
                else if (i >= second.Degree)
                {
                    diff[i] = first.coefficient[i];
                }
                else
                {
                    diff[i] = first.coefficient[i] - second.coefficient[i];
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
                multiplicand[i] = multiplier * first.coefficient[i];
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
                    compostion[j + i] += first.coefficient[i] * second.coefficient[j];
                }
            }

            var result1 = new Polynom(compostion);
            return result1;
        }
    }
}