import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_json("demo2_13_4_2024.json")

df = df[-786:]

print(df)

print(df.min())

df.plot(x="in_timestamp", y=["rel_alti", "velocity"], figsize=(20, 10))
plt.show()
