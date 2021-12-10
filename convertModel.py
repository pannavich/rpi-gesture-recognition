import tensorflow as tf

# converting tensorflow model to tflite model
converter = tf.lite.TFLiteConverter.from_saved_model("./model/")
converter.allow_custom_ops = True
tflite_model = converter.convert()
with open('./model.tflite', 'wb') as f:
  f.write(tflite_model)
  
