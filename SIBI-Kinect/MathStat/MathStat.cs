using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics;

namespace SIBI_Kinect.MathStat
{
    class MathStat
    {
        
    }

    class SignalProc {
        public static int OtsuThreshold(int[] counts, int[] x)
        {
            double total = counts.Sum();
            int len = counts.Length;
            // Pre compute for get O(1) sum query //
            double[] dp_sum = new double[len];
            dp_sum[0] = counts[0];
            double[] dp_expected = new double[len];
            dp_expected[0] = counts[0] * x[0];
            for (int ii = 1; ii < len; ii++)
            {
                dp_sum[ii] = dp_sum[ii - 1] + counts[ii];
                dp_expected[ii] = dp_expected[ii - 1] + counts[ii] * x[ii];
            }


            // Calculate otsu //
            double w_b, w_f, m_b, m_f, var_between, total_b, total_f, var_max = Double.MinValue;
            int selected_idx = -1;

            for (int t = 0; t < len - 1; t++)
            {
                total_b = dp_sum[t] - 0;
                total_f = dp_sum[len-1] - dp_sum[t];
                w_b = total_b / total;
                w_f = total_f / total;
                m_b = (dp_expected[t] - 0) / total_b;
                m_f = (dp_expected[len-1] - dp_expected[t]) / total_f;

                var_between = w_b * w_f * Math.Pow(m_b - m_f, 2);

                if (var_between > var_max) {
                    var_max = var_between;
                    selected_idx = x[t];
                }

                //Console.WriteLine(" ==== Iter :"+t+" ==== ");
                //Console.WriteLine("Var Between : "+var_between);
                
            }
            return selected_idx;
        }
    }

}
