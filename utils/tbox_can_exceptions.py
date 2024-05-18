from collections import UserDict
from dataclasses import dataclass, field
from typing import Optional

from requests.exceptions import (
    ConnectionError,
    HTTPError,
    InvalidJSONError,
    ReadTimeout,
    RequestException,
    Timeout,
)


@dataclass(kw_only=True)
class TBoxCanException(Exception):
    """Base class for all TBox CAN exceptions (Kvaser exceptions)."""

    err_code: Optional[int] = 0  # default exception is unknown connection error
    extra_msg: Optional[str] = None
    codes: UserDict = field(default_factory=UserDict)

    def __post_init__(self):
        self.codes = UserDict(  # class attribute, if not given use the default
            {
                0: "success",
                1: "xcp download failure",
                2: "xcp internal error",
                3: "network_unknown_error",
                4: "xcp flashing timeout",
            }
        )
        # print(
        #     f"{{\'header\': \'err_code\': \'{self.err_code}\', "
        #     f"\'msg\': \'{self.codes[self.err_code]}\', "
        #     f"\'extra_msg\': \'{self.extra_msg}\'}}"
        # )
