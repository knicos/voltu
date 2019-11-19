/**
 * This is the generic model for the MongoDB
 * 
 * Single collection contains the following values
 * URI: {type: String, not null}
 * data: Object
 *      the actual data
 * 
 * 
 * e.g
 * 
 * URI: 'ftl://utu.fi/stream/configurations/calibrations
 * data: {
 *      default: {
 *              board_size: [9,7]
 *              square_size: 1
 *      }
 * 
 *      HD_quality: {
 *              board_size: [5,2]
 *              square_size: 5
 *      }
 * }
 * 
 * URI: 'ftl://utu.fi/stream/configurations/disparity/name/'
 * data: {
 *      default: {
 *              name: 'default'
 *      }
 * 
 *      HD_quality: {
 *              name: 'HD_quality'
 *      }
 * 
 * }
 */


const mongoose = require('mongoose')

const configsSchema = mongoose.Schema({
    Settings: String,
    data: Object
  })

  configsSchema.set('toJSON', {
    transform: (document, returnedObject) => {
      delete returnedObject._id
      delete returnedObject.__v
    }
  })
  
module.exports = mongoose.model('configs', configsSchema)