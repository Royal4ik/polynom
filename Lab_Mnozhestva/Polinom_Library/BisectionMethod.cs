namespace Polinom_Library
{
    using System;

    public class BisectionMethod : IFindroot
    {
        public double FindRoot(Polynom polynom, double start, double end)
        {
            const double Eps = 0.000001;
            var fstart = polynom.Calculate(start);
            while (Math.Abs(start - end) > Eps)
            {
                var center = (start + end) / 2;
                var fcenter = polynom.Calculate(center);
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
    }
}