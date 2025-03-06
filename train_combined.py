import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle  

(x_mnist, y_mnist), (x_mnist_test, y_mnist_test) = keras.datasets.mnist.load_data()

x_mnist = x_mnist.astype("float32") / 255.0
x_mnist_test = x_mnist_test.astype("float32") / 255.0
x_mnist = x_mnist[..., np.newaxis]  
x_mnist_test = x_mnist_test[..., np.newaxis]

data = pd.read_csv("/kaggle/input/handwritten-datasets/A_Z Handwritten Data.csv")  
y_az = data.iloc[:, 0].values  
x_az = data.iloc[:, 1:].values.astype("float32") / 255.0  
x_az = x_az.reshape(-1, 28, 28, 1)

y_az += 10  

x_combined = np.concatenate((x_mnist, x_az), axis=0)
y_combined = np.concatenate((y_mnist, y_az), axis=0)

x_test_combined = np.concatenate((x_mnist_test, x_az[:5000]), axis=0)  
y_test_combined = np.concatenate((y_mnist_test, y_az[:5000]), axis=0)

num_classes = 36
y_combined = keras.utils.to_categorical(y_combined, num_classes)
y_test_combined = keras.utils.to_categorical(y_test_combined, num_classes)

x_combined, y_combined = shuffle(x_combined, y_combined, random_state=42)

x_train, x_val, y_train, y_val = train_test_split(x_combined, y_combined, test_size=0.2, random_state=42)

data_augmentation = keras.Sequential([
    layers.RandomRotation(0.1),
    layers.RandomZoom(0.1)
])

def build_model():
    inputs = keras.Input(shape=(28, 28, 1))  
    x = data_augmentation(inputs)
    x = layers.Conv2D(32, (3, 3), activation="relu", padding="same")(x)
    x = layers.Conv2D(32, (3, 3), activation="relu", padding="same")(x)
    x = layers.MaxPooling2D((2, 2))(x)

    x = layers.Conv2D(64, (3, 3), activation="relu", padding="same")(x)
    x = layers.Conv2D(64, (3, 3), activation="relu", padding="same")(x)
    x = layers.MaxPooling2D((2, 2))(x)

    x = layers.Flatten()(x)
    x = layers.Dense(128, activation="relu")(x)
    x = layers.Dropout(0.5)(x)
    outputs = layers.Dense(num_classes, activation="softmax")(x)

    model = keras.Model(inputs, outputs)
    return model

model = build_model()
model.compile(
    optimizer=keras.optimizers.AdamW(learning_rate=1e-3, weight_decay=1e-4),
    loss="categorical_crossentropy",
    metrics=["accuracy"]
)

model.fit(
    x_train, y_train,
    validation_data=(x_val, y_val),
    epochs=10,
    batch_size=64
)
