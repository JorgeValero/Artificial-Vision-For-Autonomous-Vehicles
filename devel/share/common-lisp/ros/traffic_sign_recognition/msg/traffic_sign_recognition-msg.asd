
(cl:in-package :asdf)

(defsystem "traffic_sign_recognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sign" :depends-on ("_package_sign"))
    (:file "_package_sign" :depends-on ("_package"))
  ))