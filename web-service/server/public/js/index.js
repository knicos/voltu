
const checkIfLoggedIn = async () => {
    const token = window.localStorage.getItem('token')
    if(!token){
        console.log("FAIL")
        return ""
    }else{
        const response = await fetch('http://localhost:8080/auth/validation', {
            method: 'POST',
            headers: {'Authorization': token}
        })
        if(response.status === 200){
            console.log("SUCCESS")
            document.getElementById('container').innerHTML = "<p>Salainen sivu</p>"
        }
    }
}

const handleLogin = () => {
    window.location.href="/google";
}