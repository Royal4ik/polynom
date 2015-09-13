// --------------------------------------------------------------------------------------------------------------------
// <copyright file="Program.cs" company="">
//   
// </copyright>
// <summary>
//   Defines the Worker type.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace Polinoms
{
    using System;
    using System.Diagnostics.CodeAnalysis;

    using Polinom_Library;

    [SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1202:ElementsMustBeOrderedByAccess", Justification = "Reviewed. Suppression is OK here."),SuppressMessage("StyleCop.CSharp.OrderingRules", "SA1202:ElementsMustBeOrderedByAccess", Justification = "Reviewed. Suppression is OK here."),SuppressMessage("StyleCop.CSharp.MaintainabilityRules", "SA1402:FileMayOnlyContainASingleClass", Justification = "Reviewed. Suppression is OK here.")]
    public class Program
    {
        [SuppressMessage("StyleCop.CSharp.NamingRules", "SA1300:ElementMustBeginWithUpperCaseLetter", Justification = "Reviewed. Suppression is OK here.")]
        public static Polynom InputPolynom()
        {
            Console.WriteLine("Введите cтепень полинома");
            var deg = Convert.ToInt16(Console.ReadLine());
            var array1 = new double[deg + 1];
            for (var i = 0; i <= deg; i++)
            {
                Console.Write("Введите коэффицент при {0}-й степени ", i);
                var koef = Convert.ToInt16(Console.ReadLine());
                array1[i] = koef;
            }

            return new Polynom(deg, array1);
        }

        public static void Main(string[] args)
        {
            var compare = -1;
            while (compare != 0)
            {
                Console.WriteLine();
                Console.WriteLine("Выберите из списка операцию с полиномом:");
                Console.WriteLine("1. Вычисление массива значений от массива аргументов");
                Console.WriteLine("2. Cложение полиномов");
                Console.WriteLine("3. Вычитание полиномов");
                Console.WriteLine("4. Умножение полинома на число");
                Console.WriteLine("5. Умножение полиномов");
                Console.WriteLine("6. Bычисление корня численным способом на заданном отрезке");
                Console.WriteLine("0. Exit");
                
                compare = Convert.ToInt16(Console.ReadLine());
                switch (compare)
                {
                    case 0:
                        {
                            break;
                        }

                    case 1:
                        {
                            Console.WriteLine("Введите полином:");
                            var polynom = InputPolynom();
                            Console.WriteLine("Введите количество элемeнтов массива значений");
                            var number = Convert.ToInt16(Console.ReadLine());
                            Console.WriteLine("Введите значения элементов");
                            var arguments = new double[number];
                            for (var i = 0; i < number; i++)
                            {
                                arguments[i] = Convert.ToDouble(Console.ReadLine());
                            }

                            Console.WriteLine();
                            Console.WriteLine("Получившиеся значения");
                            var values = polynom.Calculate(arguments);
                            for (var i = 0; i < number; i++)
                            {
                                Console.Write(values[i] + " ");
                            }

                            break;
                        }

                    case 2:
                        {
                            Console.WriteLine("Введите первый полином:");
                            var polynom1 = InputPolynom();
                            Console.WriteLine("Введите второй полином:");
                            var polynom2 = InputPolynom();
                            Console.WriteLine("Сумма введенных полином равна:");
                            var result = polynom1 + polynom2;
                            result.PrintPol();
                            break;
                        }

                    case 3:
                        {
                            Console.WriteLine("Введите первый полином(уменьшаемое):");
                            var polynom1 = InputPolynom();
                            Console.WriteLine("Введите второй полином(вычитаемое):");
                            var polynom2 = InputPolynom();
                            Console.WriteLine("Разность введенных полином равна:");
                            var result = polynom1 - polynom2;
                            result.PrintPol();
                            break;               
                        }

                    case 4:
                        {
                            Console.WriteLine("Введите полином:");
                            var polynom = InputPolynom();
                            Console.WriteLine("Введите число на которое хотите его умножить:");
                            var multiplier = Convert.ToDouble(Console.ReadLine());
                            Console.WriteLine("Умножение числа на полином равно:");
                            var result = polynom * multiplier;
                            result.PrintPol();
                            break;
                        }

                    case 5:
                        {
                            Console.WriteLine("Введите первый полином:");
                            var polynom1 = InputPolynom();
                            Console.WriteLine("Введите второй полином:");
                            var polynom2 = InputPolynom();
                            Console.WriteLine("Произведение введенных полиномов равно:");
                            var result = polynom1 * polynom2;
                            result.PrintPol();
                            break;
                        }

                    case 6:
                        {
                            Console.WriteLine("Введите начало и конец отрезка на котором находится корень");
                            var start = Convert.ToDouble(Console.ReadLine());
                            var end = Convert.ToDouble(Console.ReadLine());
                            var variable = -1;
                            while (variable != 0)
                            {
                                Console.WriteLine("Выберите метод решения:");
                                Console.WriteLine("1. Бинарный поиск");
                                Console.WriteLine("2. Метод Ньютона");
                                Console.WriteLine("0. Exit");
                                variable = Convert.ToInt16(Console.ReadLine());
                                Console.WriteLine("Введите полином");
                                IFindroot polynom;
                                switch (variable)
                                {
                                    case 1:
                                        {
                                            polynom = InputPolynom();
                                            var result = polynom.FindRoot(start, end);
                                            Console.WriteLine("Root is " + result);
                                            break;
                                        }

                                    case 2:
                                        {
                                            polynom = new Polynom2();
                                            var result = polynom.FindRoot(start, end);
                                            Console.WriteLine("Root is " + result);
                                            break;
                                        }
                                }
                            }

                            break;
                        }
                }
            }
        }
    }
}
