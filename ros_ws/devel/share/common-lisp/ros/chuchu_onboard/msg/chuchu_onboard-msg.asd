
(cl:in-package :asdf)

(defsystem "chuchu_onboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "chuchu_msg" :depends-on ("_package_chuchu_msg"))
    (:file "_package_chuchu_msg" :depends-on ("_package"))
    (:file "Int16ArrayHeader" :depends-on ("_package_Int16ArrayHeader"))
    (:file "_package_Int16ArrayHeader" :depends-on ("_package"))
  ))