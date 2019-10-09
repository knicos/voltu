const mongoose = require('mongoose')

const disparitySchema = mongoose.Schema({
    name: {type: String, default: null},
    algorithm: {type: String, default: 'libsgm'},
    use_cuda: {type: Boolean, default: true},
    minimum: {type: Number, default: 0},
    maximum: {type: Number, default: 256},
    tau: {type: Number, default: 0.0},
    gamma: {type: Number, default: 0.0},
    window_size: {type: Number, default: 5},
    sigma: {type: Number, default: 1.5},
    lambda: {type: Number, default: 8000.0}
})

disparitySchema.set('toJSON', {
    transform: (document, returnedObject) => {
      delete returnedObject._id
      delete returnedObject.__v
    }
  })
  
module.exports = mongoose.model('disparityConfig', disparitySchema)