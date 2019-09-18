const passport = require('passport');
const GoogleStrategy = require('passport-google-oauth20');
const keys = require('./keys')

passport.use(
    new GoogleStrategy({
        clientID: keys.google.clientID,
        clientSecret: keys.google.clientSecret,
        callbackURL: '/auth/google/redirect',
        passReqToCallback: true
    }, (req, accessToken, refreshToken, profile, done) => {

        return done(null, profile)
    })
)