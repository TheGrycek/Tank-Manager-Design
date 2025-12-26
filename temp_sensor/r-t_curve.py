import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit


def main():
    df_read = pd.read_csv("./r-t_curve.csv")
    df_read["R [kOhm]"] *= 1000
    df_ntc = pd.read_csv("./ntc10k_nominal.csv")

    def ntc_curve_func(r, a, b, c):
        # Steinhart - Hart formula for NTC10k thermistor
        return (1 / (a + (b * np.log(r)) + (c * (np.log(r)) ** 3))) - 273.15

    r = df_ntc["Nominal resistance"]
    t = df_ntc["Temperature"]
    initial_params = [1.0406e-3, 2.3264e-4, 8.0333e-8]

    params, covariance = curve_fit(ntc_curve_func, r, t, p0=initial_params)
    # estimated params: a = 1.14782450e-03, b = 2.31732771e-04, c = 9.16549992e-08]
    print(params)

    t_fit = ntc_curve_func(r.to_numpy(), *params)

    plt.scatter(x=df_read["T [*C]"], y=df_read["R [kOhm]"])
    plt.plot(t_fit, r, c="blue")
    # plt.scatter(x=df_ntc["Temperature"], y=df_ntc["Nominal resistance"])
    plt.plot(df_ntc["Temperature"], df_ntc["Maximum resistance"], ls="--", c="red")
    plt.plot(df_ntc["Temperature"], df_ntc["Minimum resistance"], ls="--", c="red")
    plt.xlabel("T [*C]")
    plt.ylabel("R [kOhm]")
    plt.title("R-T curve")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
