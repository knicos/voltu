const mongoose = require('mongoose')

const configSchema = mongoose.Schema({    
    board_size: Array,
    square_size: Number,
    frame_delay: Number,
    num_frames: Number,
    assume_zero_tangential_distortion: Boolean,
    fix_aspect_ratio: Boolean,
    fix_principal_point_at_center: Boolean,
    use_fisheye_model: Boolean,
    fix_k1: {type: Boolean, default: false},
    fix_k2: {type: Boolean, default: false},
    fix_k3: {type: Boolean, default: false},
    fix_k4: {type: Boolean, default: true},
    fix_k5: {type: Boolean, default: true},
    //Mongoose doesn't let you use attribute save
    //save: {type: Boolean, default: true},
    use_intrinsics: {type: Boolean, default: true},
    use_extrinsics: {type: Boolean, default: true},
    flip_vertical: {type: Boolean, default: true},
  })

  configSchema.set('toJSON', {
    transform: (document, returnedObject) => {
      returnedObject.id = returnedObject._id
      delete returnedObject._id
      delete returnedObject.__v
    }
  })
  
module.exports = mongoose.model('Config', configSchema)