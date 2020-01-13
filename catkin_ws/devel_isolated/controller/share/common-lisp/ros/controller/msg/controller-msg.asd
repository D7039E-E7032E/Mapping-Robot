
(cl:in-package :asdf)

(defsystem "controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Dirvector" :depends-on ("_package_Dirvector"))
    (:file "_package_Dirvector" :depends-on ("_package"))
    (:file "Pathvector" :depends-on ("_package_Pathvector"))
    (:file "_package_Pathvector" :depends-on ("_package"))
  ))