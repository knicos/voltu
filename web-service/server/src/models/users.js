const mongoose = require('mongoose')

const userSchema = mongoose.Schema({
    googleID: Number,
    hakaID: String
  })
  
module.exports = mongoose.model('User', userSchema)