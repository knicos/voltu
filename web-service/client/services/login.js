import axios from 'axios'
const baseUrl = 'http://localhost:8080/google'

const login = async credentials => {
  const response = await axios.post(baseUrl)
  return response.data
}

export default { login }